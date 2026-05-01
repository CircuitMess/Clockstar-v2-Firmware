# Clockstar v2 Firmware Architecture

This document provides a deep dive into the architecture and design patterns used in the Clockstar v2 firmware.

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Abstraction Layer](#hardware-abstraction-layer)
4. [Service Architecture](#service-architecture)
5. [Screen Management](#screen-management)
6. [Event System](#event-system)
7. [Threading Model](#threading-model)
8. [Memory Management](#memory-management)
9. [Power Management](#power-management)
10. [Bluetooth Architecture](#bluetooth-architecture)

## Overview

The Clockstar v2 firmware is built on ESP-IDF 5.1 and uses a layered architecture:

```
┌─────────────────────────────────────┐
│     Application Layer (Screens)     │
├─────────────────────────────────────┤
│      UI Framework (LVGL/LovyanGFX)  │
├─────────────────────────────────────┤
│       Services & Business Logic     │
├─────────────────────────────────────┤
│     Hardware Abstraction Layer      │
├─────────────────────────────────────┤
│     ESP-IDF / FreeRTOS / Hardware   │
└─────────────────────────────────────┘
```

## System Architecture

### Initialization Flow

The system initialization follows this sequence (in `main.cpp`):

1. **Hardware Detection**: Check for correct hardware (Clockstar v2 vs Bit v3)
2. **EFUSE Verification**: Verify hardware revision and PID
3. **NVS Initialization**: Initialize non-volatile storage
4. **Service Creation**: Instantiate core services in specific order
5. **Screen Loading**: Load initial lock screen
6. **UI Thread Start**: Start LVGL UI thread

### Key Components

#### 1. Pins Configuration (`Pins.hpp`)

Hardware pins are abstracted through the `Pins` class:

```cpp
enum class Pin : uint8_t {
    BtnDown, BtnUp, BtnSelect, BtnAlt,
    LedBl,      // Backlight PWM
    Buzz,       // Buzzer PWM
    BattRead,   // Battery ADC
    I2cSda, I2cScl,
    TftSck, TftMosi, TftDc, TftRst,
    Rgb_r, Rgb_g, Rgb_b,
    Imu_int1, Imu_int2
};

int pinNum = Pins::get(Pin::BtnUp);
```

This allows for hardware revision changes without modifying application code.

#### 2. Settings (`Settings/Settings.h`)

Persistent configuration stored in NVS:
- Screen brightness
- Notification sounds
- Sleep timeout
- Bluetooth pairing data

## Hardware Abstraction Layer

### Display (`Devices/Display.h`)

Wraps LovyanGFX for ST7735S TFT controller:

```cpp
class Display {
    LGFX_Device& getLGFX();  // Direct access for custom drawing
    void drawTest();          // Hardware test
};
```

Properties:
- Resolution: 128x128 pixels
- Color depth: 16-bit RGB565
- SPI interface

### Input (`Devices/Input.h`)

Handles button debouncing and event generation:

```cpp
enum Button { Up, Down, Select, Alt };

struct Data {
    Button btn;
    enum Action { Release, Press } action;
};
```

Features:
- Hardware debouncing (5ms)
- Event-driven architecture
- 20ms polling interval
- Generates events to `Facility::Input`

### IMU (`Devices/IMU.h`)

LSM6DS3 6-axis IMU (accelerometer + gyroscope):

```cpp
struct Sample {
    float accelX, accelY, accelZ;  // in g's
    float gyroX, gyroY, gyroZ;      // in deg/s
};

Sample getSample();
```

Sensor fusion available through `Fusion/` library for orientation tracking.

### Battery (`Devices/BatteryV2.h`)

Battery monitoring via ADC:

```cpp
class BatteryV2 : public Battery {
    uint8_t getLevel();        // 0-100%
    uint16_t getVoltage();     // in mV
    bool isCharging();
    bool isShutdown();         // Critical battery
};
```

Events:
- Battery level changes
- Charging state changes
- Critical battery (auto-shutdown)

### RTC (`Devices/RTC.h`)

PCF8563 real-time clock via I2C:

```cpp
struct Time {
    uint8_t hour, minute, second;
    uint8_t day, month;
    uint16_t year;
};

void setTime(const Time& time);
Time getTime();
```

## Service Architecture

Services use the Service Locator pattern via `Util/Services.h`:

```cpp
enum class Service {
    Settings, Backlight, Audio, IMU, Display,
    Input, Sleep, Status, Battery, Time, Phone
};

// Registration (usually in main.cpp)
Services.set(Service::Audio, audioInstance);

// Access
auto audio = (ChirpSystem*) Services.get(Service::Audio);
```

### Key Services

#### ChirpSystem (`Services/ChirpSystem.h`)

Audio playback via PWM-driven buzzer:

```cpp
struct Chirp {
    uint16_t startFreq;  // Hz
    uint16_t endFreq;    // Hz
    uint16_t duration;   // ms
};

void play(std::vector<Chirp> chirps);
void play(Chirp chirp);
void stop();
```

Uses a separate FreeRTOS task for non-blocking audio.

#### SleepMan (`Services/SleepMan.h`)

Power management and sleep control:

```cpp
void enAutoSleep(bool enable);  // Enable/disable auto-sleep
void wake(bool resetTimer);     // Keep awake
void shutdown();                // Power off device
```

Features:
- Configurable sleep timeout
- Wakeup on button press
- Deep sleep mode
- Shutdown animation

#### Time (`Services/Time.h`)

Time service with RTC sync:

```cpp
time_t getTime();           // Unix timestamp
void setTime(time_t time);
bool sync();                // Sync with RTC
```

Integrates with phone for time synchronization.

#### StatusCenter (`Services/StatusCenter.h`)

System status indicators:

```cpp
void blockAudio(bool block);  // Prevent notification sounds
// Status bar integration
```

## Screen Management

### LVScreen Base Class (`LV_Interface/LVScreen.h`)

All screens inherit from `LVScreen`:

```cpp
class MyScreen : public LVScreen {
protected:
    lv_group_t* inputGroup;  // For input focus

    // Lifecycle hooks
    virtual void onStarting() { }  // Before show (load resources)
    virtual void onStart() { }     // After show (start tasks)
    virtual void onStop() { }      // Before hide (cleanup)
    virtual void loop() { }        // Periodic updates

    // Transition to another screen
    void transition(std::function<std::unique_ptr<LVScreen>()> create);
};
```

### LVGL Integration (`LV_Interface/LVGL.h`)

Wraps LVGL library:

```cpp
class LVGL {
    void start();  // Start UI thread
    void stop(uint32_t timeout);
    
    void startScreen(std::function<std::unique_ptr<LVScreen>()> create);
    
    lv_disp_t* disp();  // LVGL display
};
```

Features:
- Dedicated FreeRTOS task for UI rendering
- 30ms tick rate
- Screen transition management
- Memory management

### Screen Examples

#### Simple Static Screen

```cpp
class SimpleScreen : public LVScreen {
public:
    SimpleScreen() {
        lv_obj_t* label = lv_label_create(*this);
        lv_label_set_text(label, "Hello!");
        lv_obj_center(label);
    }
};
```

#### Interactive Screen with Input

```cpp
class InteractiveScreen : public LVScreen {
    EventQueue queue;
    
    InteractiveScreen() : queue(4) {
        // Create UI...
    }
    
    void onStart() override {
        Events::listen(Facility::Input, &queue);
    }
    
    void onStop() override {
        Events::unlisten(&queue);
    }
    
    void loop() override {
        Event event;
        if(queue.get(event, 0)) {
            // Handle events
        }
    }
};
```

#### Game/Animation Screen with Thread

```cpp
class GameScreen : public LVScreen {
    ThreadedClosure gameThread;
    std::atomic_bool running{false};
    
    GameScreen() : gameThread([this](){ gameLoop(); }, "Game", 4096) { }
    
    void onStart() override {
        running = true;
        gameThread.start();
    }
    
    void onStop() override {
        running = false;
        gameThread.stop(0);
    }
    
    void gameLoop() {
        while(running) {
            // Update game state
            vTaskDelay(pdMS_TO_TICKS(16));  // 60 FPS
        }
    }
};
```

## Event System

### Event Architecture (`Util/Events.h`)

Centralized event dispatch system:

```cpp
enum class Facility {
    Input,      // Button events
    Phone,      // Bluetooth/phone events
    Battery,    // Battery status changes
    Sleep       // Sleep/wake events
};

struct Event {
    Facility facility;
    void* data;
};

class EventQueue {
public:
    EventQueue(size_t size);
    bool get(Event& event, uint32_t timeout);
};

// Subscribe to events
Events::listen(Facility::Input, &queue);

// Unsubscribe
Events::unlisten(&queue);

// Post events (internal use)
Events::post(Facility::Input, &inputData);
```

### Event Flow

```
Button Press → Input::scan() → Events::post() → EventQueue → Screen::loop()
```

## Threading Model

### FreeRTOS Tasks

The firmware uses multiple tasks:

| Task | Priority | Stack | Purpose |
|------|----------|-------|---------|
| main | 1 | Auto | Initialization only (self-deletes) |
| LVGL | 5 | 4KB | UI rendering and input |
| Input | 5 | 2KB | Button scanning |
| Battery | 3 | 2KB | Battery monitoring |
| Audio | 5 | 2KB | Buzzer control |
| BLE | Variable | Various | Bluetooth stack |

### Synchronization

- **Mutexes**: Protect shared resources (LVGL, I2C)
- **Semaphores**: Signal completion (audio, screen transitions)
- **Queues**: Inter-task communication (events)
- **Atomics**: Lock-free flags

### Thread Safety

LVGL is **not thread-safe**. UI updates must happen in LVGL task:

```cpp
// WRONG: Direct UI update from another task
void otherTask() {
    lv_label_set_text(label, "Bad!");  // CRASH!
}

// CORRECT: Use event system
void otherTask() {
    Events::post(Facility::Custom, &data);
}

void Screen::loop() {
    Event event;
    if(queue.get(event, 0)) {
        lv_label_set_text(label, "Good!");  // Safe in LVGL task
    }
}
```

## Memory Management

### Memory Constraints

ESP32-S3 memory:
- SRAM: ~512KB (shared with WiFi/BLE)
- PSRAM: 2MB (not used by default)
- Flash: 4MB

### Stack vs Heap

- Use stack for small, short-lived objects
- Use heap for:
  - Large buffers
  - Long-lived objects
  - Variable-sized data

### LVGL Memory

LVGL has its own heap (configured in `lv_conf.h`):
- Default: 32KB
- Used for UI objects, styles, images
- Monitor with `lv_mem_monitor()`

### Best Practices

```cpp
// Good: Stack allocation
void function() {
    char buffer[64];  // Small, short-lived
}

// Good: Heap for large/persistent objects
class Screen {
    std::vector<int> largeData;  // Managed by std::vector
};

// Avoid: Large stack allocations
void badFunction() {
    uint8_t hugeBuffer[10000];  // May overflow stack!
}

// Good: Static allocation for large constant data
static const uint8_t imageData[] PROGMEM = { /* ... */ };
```

## Power Management

### Sleep States

1. **Active**: All systems running
2. **Screen Off**: Backlight off, CPU running
3. **Light Sleep**: CPU paused, wake on button
4. **Deep Sleep**: Most systems off, wake on button (not implemented)
5. **Shutdown**: Complete power off

### Power Flow

```
User Activity → SleepMan::wake() → Reset Timer
                                    ↓
                            Timer Expires (30s)
                                    ↓
                            Screen Fade Out
                                    ↓
                            Light Sleep Mode
```

### Preventing Sleep

```cpp
void onStart() override {
    auto sleep = (SleepMan*) Services.get(Service::Sleep);
    sleep->enAutoSleep(false);  // Disable during game/activity
}

void onStop() override {
    auto sleep = (SleepMan*) Services.get(Service::Sleep);
    sleep->enAutoSleep(true);   // Re-enable when done
}
```

## Bluetooth Architecture

### BLE Stack (`BLE/`)

Three main components:

#### GAP (Generic Access Profile) - `BLE/GAP.h`

Connection management:
- Advertising
- Scanning
- Connection establishment

#### Server - `BLE/Server.h`

GATT server for phone connectivity:
- Current time service
- Battery service
- Custom services

#### Client - `BLE/Client.h`

GATT client for phone services:
- ANCS (Apple Notification Center Service) for iOS
- Custom services for Android

### Phone Integration (`Notifs/Phone.h`)

High-level phone interface:

```cpp
enum class PhoneType { None, Android, iOS };

PhoneType getType();
bool isConnected();
void findPhone();  // Trigger phone alarm
```

Handles:
- Pairing
- Time sync
- Notifications
- Find my phone

## File System

### SPIFFS (`LV_Interface/FSLVGL.h`)

Images and assets stored in SPIFFS partition:

```
spiffs_image/
├── bg.bin              # Background images
├── menu/
│   ├── find            # Menu icons
│   ├── level
│   └── ...
└── ...
```

Access via LVGL:

```cpp
lv_obj_set_style_bg_img_src(obj, "S:/bg.bin", 0);
```

Image format: Raw binary (RGB565 or indexed)

## Development Tips

### Adding a New Service

1. Create service class
2. Register in `main.cpp` during init:
   ```cpp
   auto myService = new MyService();
   Services.set(Service::Custom, myService);
   ```
3. Access from screens:
   ```cpp
   auto srv = (MyService*) Services.get(Service::Custom);
   ```

### Adding a New Screen

1. Create `MyScreen.h` and `MyScreen.cpp` in `Screens/`
2. Inherit from `LVScreen`
3. Implement UI in constructor
4. Add lifecycle methods as needed
5. Add to menu or transition from another screen

### Debugging

Enable detailed logging:

```cpp
esp_log_level_set("MyTag", ESP_LOG_DEBUG);
ESP_LOGD("MyTag", "Value: %d", value);
```

Monitor memory:

```cpp
ESP_LOGI("Mem", "Free heap: %d", esp_get_free_heap_size());

lv_mem_monitor_t mon;
lv_mem_monitor(&mon);
ESP_LOGI("LVGL", "Used: %d, Free: %d", mon.used_size, mon.free_size);
```

### Performance Optimization

- Minimize heap allocations in hot paths
- Use object pools for frequent allocations
- Cache computed values
- Reduce LVGL object count
- Optimize display refreshes
- Profile with ESP-IDF tools

## Conclusion

The Clockstar v2 firmware provides a solid foundation for building smartwatch applications. Understanding this architecture will help you:

- Create robust applications
- Integrate with existing services
- Debug issues effectively
- Contribute to the firmware

For more information, see:
- [CUSTOMIZATION.md](../CUSTOMIZATION.md) - Getting started guide
- [tutorials/PONG_GAME.md](tutorials/PONG_GAME.md) - Practical example
- ESP-IDF documentation
- LVGL documentation
