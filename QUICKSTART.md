# Quick Start Guide - Clockstar v2 Firmware Customization

Get started customizing your Clockstar v2 firmware in under 30 minutes!

## Prerequisites

- ✅ Clockstar v2 device
- ✅ Computer (Windows, Linux, or macOS)
- ✅ USB cable
- ✅ Basic C++ knowledge

## Step 1: Set Up Your Environment (10 minutes)

### Install ESP-IDF

1. Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
2. Install ESP-IDF version 5.1, commit `3a45d4e`

```bash
git clone -b v5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout 3a45d4e949a174e8829a2e4c86c421b030ceac5a
./install.sh
. ./export.sh
```

## Step 2: Clone and Setup Project (5 minutes)

```bash
# Clone the repository
git clone --recursive https://github.com/ercasta/Clockstar-v2-Firmware.git
cd Clockstar-v2-Firmware

# Apply ESP-IDF patch
cd /path/to/esp-idf
git apply /path/to/Clockstar-v2-Firmware/ESP-IDF.patch

# Apply LovyanGFX patch
cd /path/to/Clockstar-v2-Firmware/components/LovyanGFX
git apply ../../LovyanGFX.patch
cd ../..
```

## Step 3: Build and Flash (5 minutes)

```bash
# Build the firmware
idf.py build

# Flash to your Clockstar v2
idf.py -p /dev/ttyACM0 flash

# Monitor output (optional)
idf.py -p /dev/ttyACM0 monitor
```

**Note**: Replace `/dev/ttyACM0` with your device port (e.g., `COM6` on Windows)

## Step 4: Create Your First App (10 minutes)

Let's create a simple "Hello World" screen!

### Create the Header File

Create `main/src/Screens/HelloWorld.h`:

```cpp
#ifndef CLOCKSTAR_FIRMWARE_HELLOWORLD_H
#define CLOCKSTAR_FIRMWARE_HELLOWORLD_H

#include "../LV_Interface/LVScreen.h"
#include "Util/Events.h"

class HelloWorld : public LVScreen {
public:
    HelloWorld();
    ~HelloWorld() override;

private:
    void onStart() override;
    void loop() override;

    lv_obj_t* label;
    EventQueue queue;
    int counter = 0;
};

#endif
```

### Create the Implementation

Create `main/src/Screens/HelloWorld.cpp`:

```cpp
#include "HelloWorld.h"
#include "Util/Services.h"
#include "Devices/Input.h"
#include "MainMenu/MainMenu.h"

HelloWorld::HelloWorld() : queue(4) {
    // Create a label
    label = lv_label_create(*this);
    lv_label_set_text(label, "Hello World!\nPress Select");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);
}

HelloWorld::~HelloWorld() {
    Events::unlisten(&queue);
}

void HelloWorld::onStart() {
    Events::listen(Facility::Input, &queue);
}

void HelloWorld::loop() {
    Event event;
    if(queue.get(event, 0)) {
        if(event.facility == Facility::Input) {
            auto data = (Input::Data*) event.data;
            
            if(data->action == Input::Data::Press) {
                if(data->btn == Input::Button::Alt) {
                    // Go back to main menu
                    transition([](){ return std::make_unique<MainMenu>(); });
                } else if(data->btn == Input::Button::Select) {
                    // Increment counter
                    counter++;
                    char buf[32];
                    snprintf(buf, sizeof(buf), "Count: %d\nAlt to exit", counter);
                    lv_label_set_text(label, buf);
                }
            }
        }
    }
}
```

### Build and Test

```bash
idf.py build flash monitor
```

## What's Next?

### 📚 Learn More

- **[CUSTOMIZATION.md](CUSTOMIZATION.md)** - Comprehensive customization guide
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - Deep dive into firmware architecture
- **[docs/tutorials/PONG_GAME.md](docs/tutorials/PONG_GAME.md)** - Build a complete Pong game

### 🎮 Try the Examples

The repository includes working examples:

1. **Level Tool** (`main/src/Screens/Level.cpp`) - Bubble level using IMU
2. **Theremin** (`main/src/Screens/Theremin/`) - Musical instrument
3. **Pong Game** (`main/src/Screens/PongGame.cpp`) - Classic Pong with motion controls

### 🛠️ Common Customizations

#### Add a Menu Item

Edit `main/src/Screens/MainMenu/MainMenu.cpp` to add your screen to the main menu.

#### Use the IMU (Motion Sensor)

```cpp
auto imu = (IMU*) Services.get(Service::IMU);
IMU::Sample sample = imu->getSample();
// sample.accelX, .accelY, .accelZ
```

#### Play Sounds

```cpp
auto audio = (ChirpSystem*) Services.get(Service::Audio);
audio->play(Chirp{ .startFreq = 440, .endFreq = 880, .duration = 100 });
```

#### Read Buttons

```cpp
// Via events (recommended)
Events::listen(Facility::Input, &queue);

// Or check state directly
auto input = (Input*) Services.get(Service::Input);
bool pressed = input->getState(Input::Button::Up);
```

## Debugging Tips

### View Serial Output

```bash
idf.py -p /dev/ttyACM0 monitor
```

Press `Ctrl+]` to exit.

### Add Debug Logging

```cpp
#include <esp_log.h>

ESP_LOGI("MyApp", "Hello from my app!");
ESP_LOGD("MyApp", "Debug: value = %d", value);
ESP_LOGE("MyApp", "Error occurred!");
```

### Check Memory Usage

```cpp
ESP_LOGI("Memory", "Free heap: %d bytes", esp_get_free_heap_size());
```

## Troubleshooting

### Build Fails

- ✅ Ensure ESP-IDF 5.1 is installed and sourced (`. ./export.sh`)
- ✅ Apply both patches (ESP-IDF.patch and LovyanGFX.patch)
- ✅ Clone with `--recursive` to get submodules

### Flash Fails

- ✅ Check correct port (`ls /dev/tty*` on Linux/Mac, Device Manager on Windows)
- ✅ Ensure Clockstar is connected and powered on
- ✅ Try different USB cable/port
- ✅ Hold reset button during flash

### Code Doesn't Run

- ✅ Check serial monitor for error messages
- ✅ Verify EFUSE is correctly programmed (automatic on first boot)
- ✅ Ensure battery is charged

## Getting Help

- 📖 **Documentation**: See the `docs/` folder
- 🐛 **Bug Reports**: [GitHub Issues](https://github.com/ercasta/Clockstar-v2-Firmware/issues)
- 💬 **Questions**: Check existing issues or create a new one

## Resources

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [LVGL Documentation](https://docs.lvgl.io/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/)
- [C++ Reference](https://en.cppreference.com/)

## Summary

You now know how to:
- ✅ Set up the development environment
- ✅ Build and flash firmware
- ✅ Create a basic screen
- ✅ Handle button input
- ✅ Use services (IMU, Audio, etc.)

Happy hacking! 🚀
