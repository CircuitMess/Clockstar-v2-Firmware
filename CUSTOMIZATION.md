# Clockstar v2 Firmware Customization Guide

Welcome to the Clockstar v2 firmware customization guide! This document will help you understand and customize the firmware to create your own applications and features.

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Firmware Architecture](#firmware-architecture)
4. [Getting Started](#getting-started)
5. [Creating Your First Application](#creating-your-first-application)
6. [Tutorials](#tutorials)
7. [API Reference](#api-reference)
8. [Best Practices](#best-practices)

## Introduction

Clockstar v2 is a smartwatch built on the ESP32-S3 platform. The firmware uses:
- **ESP-IDF 5.1**: Core framework for ESP32 development
- **LVGL (LittlevGL)**: Graphics library for UI rendering
- **LovyanGFX**: Display driver library
- **FreeRTOS**: Real-time operating system

### Hardware Features

The Clockstar v2 smartwatch includes:
- **Display**: 128x128 ST7735S TFT display
- **Input**: 4 physical buttons (Up, Down, Select, Alt)
- **Sensors**: IMU (accelerometer/gyroscope) for motion detection
- **Audio**: Buzzer for sound/chirp generation
- **Connectivity**: Bluetooth LE for phone pairing
- **Battery**: Rechargeable battery with charging circuit
- **RGB LEDs**: Status indication LEDs
- **RTC**: Real-time clock for timekeeping

## Prerequisites

Before you start customizing the firmware, ensure you have:

1. **Development Environment**:
   - ESP-IDF 5.1 installed (commit 3a45d4e)
   - Git for version control
   - A C++ compiler (included with ESP-IDF)

2. **Hardware**:
   - Clockstar v2 device
   - USB cable for programming/debugging

3. **Knowledge**:
   - Basic C++ programming
   - Understanding of embedded systems
   - Familiarity with FreeRTOS concepts (tasks, queues, semaphores)

For detailed build instructions, see the main [README.md](README.md).

## Firmware Architecture

The firmware follows a modular architecture with clear separation of concerns:

### Directory Structure

```
main/
├── main.cpp                 # Entry point (app_main)
├── src/
│   ├── Devices/            # Hardware abstraction layer
│   │   ├── Display.cpp     # Display driver
│   │   ├── Input.cpp       # Button input handling
│   │   ├── IMU.cpp         # Motion sensor
│   │   ├── Battery.cpp     # Battery management
│   │   └── RTC.cpp         # Real-time clock
│   ├── Services/           # System services
│   │   ├── ChirpSystem.cpp # Audio/buzzer control
│   │   ├── SleepMan.cpp    # Power management
│   │   └── Time.cpp        # Time service
│   ├── Screens/            # UI screens/applications
│   │   ├── MainMenu/       # Main menu screen
│   │   ├── Level/          # Level/bubble app
│   │   ├── Theremin/       # Theremin app
│   │   └── Settings/       # Settings screen
│   ├── LV_Interface/       # LVGL integration
│   │   ├── LVGL.cpp        # LVGL wrapper
│   │   └── LVScreen.h      # Base screen class
│   ├── BLE/                # Bluetooth functionality
│   ├── Notifs/             # Notifications system
│   └── Util/               # Utility classes
```

### Key Concepts

#### 1. Services

The firmware uses a service locator pattern for accessing system-wide services:

```cpp
// Access services
auto audio = (ChirpSystem*) Services.get(Service::Audio);
auto input = (Input*) Services.get(Service::Input);
auto imu = (IMU*) Services.get(Service::IMU);
```

Available services:
- `Service::Audio` - Sound/buzzer control
- `Service::Input` - Button input
- `Service::Display` - Display access
- `Service::IMU` - Motion sensor
- `Service::Battery` - Battery status
- `Service::Time` - Time/clock
- `Service::Sleep` - Power management
- `Service::Phone` - Bluetooth phone connection
- `Service::Settings` - User settings

#### 2. Screens (LVScreen)

All UI screens inherit from `LVScreen` base class. Screens are managed by the LVGL system and have lifecycle methods:

```cpp
class MyScreen : public LVScreen {
public:
    MyScreen();
    ~MyScreen() override;

private:
    void onStarting() override;  // Called before screen is shown
    void onStart() override;     // Called when screen becomes active
    void onStop() override;      // Called when screen is being closed
    void loop() override;        // Called periodically (main loop)
};
```

#### 3. Event System

The firmware uses an event-driven architecture:

```cpp
EventQueue queue;

// In constructor/onStart:
Events::listen(Facility::Input, &queue);

// In loop():
Event event;
if(queue.get(event, 0)) {
    if(event.facility == Facility::Input) {
        auto data = (Input::Data*) event.data;
        // Handle button press/release
    }
}
```

#### 4. Input Handling

Four buttons are available:
- `Input::Button::Up` - Navigate up
- `Input::Button::Down` - Navigate down
- `Input::Button::Select` - Confirm/select
- `Input::Button::Alt` - Alternative action/back

## Getting Started

### 1. Build and Flash

First, ensure you can build and flash the stock firmware:

```bash
# Clone repository with submodules
git clone --recursive https://github.com/ercasta/Clockstar-v2-Firmware.git
cd Clockstar-v2-Firmware

# Apply ESP-IDF patch (from ESP-IDF installation directory)
cd /path/to/esp-idf
git apply /path/to/Clockstar-v2-Firmware/ESP-IDF.patch

# Apply LovyanGFX patch
cd /path/to/Clockstar-v2-Firmware/components/LovyanGFX
git apply ../../LovyanGFX.patch

# Build
cd /path/to/Clockstar-v2-Firmware
idf.py build

# Flash
idf.py -p /dev/ttyACM0 flash
```

### 2. Monitor Serial Output

To see debug output:

```bash
idf.py -p /dev/ttyACM0 monitor
```

Press Ctrl+] to exit the monitor.

## Creating Your First Application

Let's create a simple "Hello World" screen that displays text and responds to button presses.

### Step 1: Create Screen Files

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
    void onStop() override;
    void loop() override;

    lv_obj_t* bg;
    lv_obj_t* label;
    EventQueue queue;
    int counter = 0;
};

#endif // CLOCKSTAR_FIRMWARE_HELLOWORLD_H
```

Create `main/src/Screens/HelloWorld.cpp`:

```cpp
#include "HelloWorld.h"
#include "Util/Services.h"
#include "Devices/Input.h"
#include "MainMenu/MainMenu.h"

HelloWorld::HelloWorld() : queue(4) {
    // Create background
    bg = lv_obj_create(*this);
    lv_obj_set_size(bg, 128, 128);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
    
    // Create label
    label = lv_label_create(bg);
    lv_label_set_text(label, "Hello World!\nPress buttons");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);
}

HelloWorld::~HelloWorld() {
    Events::unlisten(&queue);
}

void HelloWorld::onStart() {
    Events::listen(Facility::Input, &queue);
}

void HelloWorld::onStop() {
    Events::unlisten(&queue);
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
                    return;
                } else if(data->btn == Input::Button::Select) {
                    // Increment counter
                    counter++;
                    char buf[32];
                    snprintf(buf, sizeof(buf), "Count: %d", counter);
                    lv_label_set_text(label, buf);
                }
            }
        }
    }
}
```

### Step 2: Add to Main Menu

Edit `main/src/Screens/MainMenu/MainMenu.cpp`:

1. Add include at the top:
```cpp
#include "Screens/HelloWorld.h"
```

2. Add menu item handling in the `onClick()` function (add a new case).

### Step 3: Build and Test

```bash
idf.py build flash monitor
```

## Tutorials

Detailed tutorials are available in the `docs/tutorials/` directory:

- **[Pong Game Tutorial](docs/tutorials/PONG_GAME.md)** - Create a classic Pong game with motion controls
- More tutorials coming soon!

## API Reference

### Display

```cpp
auto disp = (Display*) Services.get(Service::Display);
LGFX_Device& lgfx = disp->getLGFX();

// Display is 128x128 pixels
// Use LVGL for UI elements (recommended)
// Or use LovyanGFX for direct pixel manipulation
```

### Input (Buttons)

```cpp
auto input = (Input*) Services.get(Service::Input);

// Check current button state
bool isPressed = input->getState(Input::Button::Up);

// Or use event system (recommended)
Events::listen(Facility::Input, &queue);
```

### IMU (Motion Sensor)

```cpp
auto imu = (IMU*) Services.get(Service::IMU);

IMU::Sample sample = imu->getSample();
// sample.accelX, sample.accelY, sample.accelZ (in g's)
// sample.gyroX, sample.gyroY, sample.gyroZ (in deg/s)
```

### Audio (Buzzer)

```cpp
auto audio = (ChirpSystem*) Services.get(Service::Audio);

// Play a single tone
audio->play(Chirp{ .startFreq = 440, .endFreq = 880, .duration = 100 });

// Play a sequence
audio->play({
    Chirp{ .startFreq = 440, .endFreq = 440, .duration = 100 },
    Chirp{ .startFreq = 0, .endFreq = 0, .duration = 50 },
    Chirp{ .startFreq = 880, .endFreq = 880, .duration = 100 }
});
```

### Sleep Management

```cpp
auto sleep = (SleepMan*) Services.get(Service::Sleep);

// Prevent auto-sleep (e.g., during games)
sleep->enAutoSleep(false);

// Re-enable auto-sleep
sleep->enAutoSleep(true);

// Keep screen awake temporarily
sleep->wake(false);
```

## Best Practices

### 1. Memory Management

- Use stack allocation when possible
- Be mindful of FreeRTOS task stack sizes
- Free resources in destructors
- Use smart pointers for complex ownership

### 2. Threading

- UI updates must be done from LVGL thread
- Use `EventQueue` for inter-task communication
- Create separate tasks for blocking operations
- Use proper synchronization (semaphores, mutexes)

### 3. Power Management

- Disable auto-sleep during active applications
- Re-enable when appropriate
- Consider battery impact of continuous sensor reading

### 4. UI Design

- Screen size is 128x128 pixels
- Use the existing theme for consistent look
- Cache images to reduce flash reads
- Use LVGL built-in widgets when possible

### 5. Debugging

- Use `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE` for logging
- Monitor serial output during development
- Check return values and handle errors
- Use watchdog timers for critical tasks

### 6. Code Organization

- Keep screen logic in separate files
- Use services for shared functionality
- Follow existing naming conventions
- Document complex algorithms

## Getting Help

- **GitHub Issues**: Report bugs or request features
- **ESP-IDF Documentation**: https://docs.espressif.com/projects/esp-idf/
- **LVGL Documentation**: https://docs.lvgl.io/

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Follow the existing code style
4. Test your changes thoroughly
5. Submit a pull request

## License

See the LICENSE file in the repository root.
