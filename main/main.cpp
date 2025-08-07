#include <driver/gpio.h>
#include <nvs_flash.h>
#include "Settings/Settings.h"
#include "Pins.hpp"
#include "Periph/I2C.h"
#include "Periph/PinOut.h"
#include "Periph/Bluetooth.h"
#include "Devices/Battery.h"
#include "Devices/Display.h"
#include "Devices/Input.h"
#include "Devices/IMU.h"
#include "BLE/GAP.h"
#include "BLE/Client.h"
#include "BLE/Server.h"
#include "Notifs/Phone.h"
#include "LV_Interface/LVGL.h"
#include "LV_Interface/FSLVGL.h"
#include "LV_Interface/InputLVGL.h"
#include <lvgl/lvgl.h>
#include "Theme/theme.h"
#include "Util/Services.h"
#include "Services/BacklightBrightness.h"
#include "Services/ChirpSystem.h"
#include "Services/Time.h"
#include "Services/StatusCenter.h"
#include "Services/SleepMan.h"
#include "Screens/ShutdownScreen.h"
#include "Screens/Lock/LockScreen.h"
#include "JigHWTest/JigHWTest.h"
#include "Util/Notes.h"
#include "Devices/BatteryV2.h"
#include <Util/EfuseMeta.h>

LVGL* lvgl;
BacklightBrightness* bl;
SleepMan* sleepMan;

void shutdown(){
	lvgl->stop(0);
	lvgl->startScreen([](){ return std::make_unique<ShutdownScreen>(); });
	lv_timer_handler();
	sleepMan->wake(true);
	if(!bl->isOn()){
		bl->fadeIn();
	}
	vTaskDelay(SleepMan::ShutdownTime - 1000);
	sleepMan->shutdown();
}

/**
 * Since Clockstar v2 has specific I2C periphery compared to Bit v3,
 * we can determine which hardware this is running on.
 */
bool checkClockstarV2(){
	const gpio_num_t ClockstarI2C_SDA = GPIO_NUM_4;
	const gpio_num_t ClockstarI2C_SCL = GPIO_NUM_5;
	const uint8_t ClockstarIMU_Address = 0x6A;

	I2C clockI2C(I2C_NUM_0, ClockstarI2C_SDA, ClockstarI2C_SCL);

	return clockI2C.probe(ClockstarIMU_Address) == ESP_OK;
}

/**
 * Necessary to ascertain if this code is potentially running on a Bit v3 board,
 * since both Bit v3 and Clockstar v2 were originally fused with PID 0x0008.
 *
 * Differentiation is done using a PID addition in EFUSE_BLK4:
 * 0x00 - Bit v3
 * 0x01 - Clockstar v2
 */
void resolvePIDConflicts(){

	static constexpr esp_efuse_desc_t PIDBlock = { EFUSE_BLK4, 0, 8 };
	static constexpr const esp_efuse_desc_t* PID_Blob[] = { &PIDBlock, nullptr };
	static constexpr uint8_t Block4_Expected = 0x01;
	uint8_t pidAddition = 0;

	auto err = esp_efuse_read_field_blob((const esp_efuse_desc_t**) PID_Blob, &pidAddition, 8);
	if(err != ESP_OK){
		ESP_ERROR_CHECK_WITHOUT_ABORT(err);
		while(1){
			ESP_LOGE("main", "couldn't read EFUSE_BLK4");
			delayMillis(1000);
		}
	}
	ESP_LOGI("main", "read from block 4: %d", pidAddition);

	switch(pidAddition){
		case 0:
			if(!checkClockstarV2()){
				while(1){
					ESP_LOGE("main", "Not running on Clockstar v2");
					delayMillis(1000);
				}
			}

			err = esp_efuse_write_field_blob((const esp_efuse_desc_t**) PID_Blob, &Block4_Expected, 8);
			if(err != ESP_OK){
				ESP_ERROR_CHECK_WITHOUT_ABORT(err);
				while(1){
					ESP_LOGE("main", "couldn't write EFUSE_BLK4");
					delayMillis(1000);
				}
			}
			ESP_LOGI("main", "fused %d to BLK4", Block4_Expected);
			return;
		case Block4_Expected:
			return;
		default:
			while(1){
				ESP_LOGE("main", "EFUSE_BLK4: %d, expected: %d", pidAddition, Block4_Expected);
				delayMillis(1000);
			}
	}
}

void init(){
	esp_log_level_set("main", ESP_LOG_DEBUG);

	if(JigHWTest::checkJig()){
		printf("Jig\n");
		Pins::setLatest();
		auto test = new JigHWTest();
		test->start();
		vTaskDelete(nullptr);
	}else{
		printf("Hello\n");
	}

	if(!EfuseMeta::check()){
		while(true){
			vTaskDelay(1000);
			EfuseMeta::log();
		}
	}

	resolvePIDConflicts();


	uint8_t rev = 0;
	if(!EfuseMeta::readRev(rev)){
		while(true){
			vTaskDelay(1000);
			printf("Failed to read hardware revision.");
		}
	}

	gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM);

	auto ret = nvs_flash_init();
	if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	auto settings = new Settings();
	Services.set(Service::Settings, settings);

	auto blPwm = new PWM(Pins::get(Pin::LedBl), LEDC_CHANNEL_1, true);
	blPwm->detach();
	bl = new BacklightBrightness(blPwm);
	Services.set(Service::Backlight, bl);

	auto buzzPwm = new PWM(Pins::get(Pin::Buzz), LEDC_CHANNEL_0);
	auto audio = new ChirpSystem(*buzzPwm);
	Services.set(Service::Audio, audio);

	auto i2c = new I2C(I2C_NUM_0, (gpio_num_t) Pins::get(Pin::I2cSda), (gpio_num_t) Pins::get(Pin::I2cScl));
	auto imu = new IMU(*i2c);
	Services.set(Service::IMU, imu);

	auto disp = new Display();
	Services.set(Service::Display, disp);

	auto input = new Input();
	Services.set(Service::Input, input);

	lvgl = new LVGL(*disp);
	auto theme = theme_init(lvgl->disp());
	lv_disp_set_theme(lvgl->disp(), theme);

	auto lvglInput = new InputLVGL();
	auto fs = new FSLVGL('S');

	sleepMan = new SleepMan(*lvgl);
	Services.set(Service::Sleep, sleepMan);

	auto status = new StatusCenter();
	Services.set(Service::Status, status);

	auto adc = new ADC(ADC_UNIT_1);

	Battery* battery; // Battery is doing shutdown
	if(rev == 0){
		while(true){
			vTaskDelay(1000);
			EfuseMeta::log();
		}
	}else{
		battery = new BatteryV2(*adc);
	}

	if(battery->isShutdown()) return; // Stop initialization if battery is critical
	Services.set(Service::Battery, battery);

	auto rtc = new RTC(*i2c);
	auto time = new Time(*rtc);
	Services.set(Service::Time, time); // Time service is required as soon as Phone is up

	auto bt = new Bluetooth();
	auto gap = new BLE::GAP();
	auto client = new BLE::Client(gap);
	auto server = new BLE::Server(gap);
	auto phone = new Phone(server, client);
	server->start();
	Services.set(Service::Phone, phone);

	FSLVGL::loadCache();

	// Load start screen here
	lvgl->startScreen([](){ return std::make_unique<LockScreen>(); });

	if(settings->get().notificationSounds){
		audio->play({
							Chirp{ .startFreq = NOTE_E4, .endFreq = NOTE_GS4, .duration = 100 },
							Chirp{ .startFreq = 0, .endFreq = 0, .duration = 200 },
							Chirp{ .startFreq = NOTE_GS4, .endFreq = NOTE_B4, .duration = 100 },
							Chirp{ .startFreq = 0, .endFreq = 0, .duration = 200 },
							Chirp{ .startFreq = NOTE_B4, .endFreq = NOTE_E5, .duration = 100 }
					});
	}

	// Start UI thread after initialization
	lvgl->start();

	bl->fadeIn();

	// Start Battery scanning after everything else, otherwise Critical
	// Battery event might come while initialization is still in progress
	battery->begin();
}

extern "C" void app_main(void){
	init();

	vTaskDelete(nullptr);
}
