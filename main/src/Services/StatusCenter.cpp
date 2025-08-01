#include "StatusCenter.h"
#include "Util/Services.h"
#include "SleepMan.h"
#include "Pins.hpp"

StatusCenter::StatusCenter() : Threaded("Status", 2048), events(12),
chirp(*((ChirpSystem*) Services.get(Service::Audio))),
settings(*((Settings*) Services.get(Service::Settings)))
{
	Events::listen(Facility::Phone, &events);
	Events::listen(Facility::Battery, &events);

	auto pwmR = new PWM(Pins::get(Pin::Rgb_r), LEDC_CHANNEL_2, true);
	auto pwmG = new PWM(Pins::get(Pin::Rgb_g), LEDC_CHANNEL_3, true);
	auto pwmB = new PWM(Pins::get(Pin::Rgb_b), LEDC_CHANNEL_4, true);

	led = new RGBLEDController(*pwmR, *pwmG, *pwmB);
	led->begin();

	updateLED();
	start();
}

void StatusCenter::loop(){
	Event evt;
	if(!events.get(evt, portMAX_DELAY)) return;

	if(evt.facility == Facility::Phone){
		auto data = (Phone::Event*) evt.data;
		processPhone(*data);
	}else if(evt.facility == Facility::Battery){
		auto data = (Battery::Event*) evt.data;
		processBatt(*data);
	}

	free(evt.data);
}

void StatusCenter::processPhone(const Phone::Event& evt){
	auto phone = (Phone*) Services.get(Service::Phone);
	hasNotifs = !phone->getNotifs().empty();

	if((evt.action == Phone::Event::Added || evt.action == Phone::Event::Changed)){
		if(settings.get().notificationSounds && !audioBlocked){
			beep();
		}

		if(settings.get().ledEnable){
			blink();
		}
	}
}

void StatusCenter::processBatt(const Battery::Event& evt){
	if(evt.action == Battery::Event::Charging){
		if(evt.chargeStatus != Battery::ChargingState::Unplugged){
			battState = Charging;
		}
	}else{
		auto level = evt.level;

		if(level == Battery::Critical){
			shutdown();
			stop(0);
			return;
		}else if(level == Battery::VeryLow){
			battState = Empty;
		}else{
			battState = Ok;
		}
	}

	updateLED();
}

void StatusCenter::updateLED(){
	if(settings.get().ledEnable == false){
		led->clear();
		return;
	}

	if(battState == Empty){
		led->blinkContinuous({ 255, 0, 0 }, -1, 100, 3000);
	}else if(battState == Charging){
		led->breathe({ 100, 250, 0 }, { 150, 150, 0 }, 6000);
	}else if(battState == Ok){
		led->clear();
	}
}

void StatusCenter::blockAudio(bool block){
	this->audioBlocked = block;
}

void StatusCenter::blink(){
	led->blinkTwice({ 0, 0, 255 });
}

void StatusCenter::beep(){
	chirp.play({
		Chirp{ .startFreq = 400, .endFreq = 600, .duration = 50 },
		Chirp{ .startFreq = 0, .endFreq = 0, .duration = 100 },
		Chirp{ .startFreq = 600, .endFreq = 400, .duration = 50 }
	});
}

void StatusCenter::shutdown(){
	led->setSolid({ 150, 0, 0 });
	vTaskDelay(SleepMan::ShutdownTime-100);

	led->breathe({ 150, 0, 0 }, { 0, 0, 0 }, 1000);
	vTaskDelay(500);

	led->clear();
}
