# Clockstar v2 Firmware Documentation

Welcome to the Clockstar v2 firmware documentation! This directory contains comprehensive guides to help you understand and customize the firmware.

## 📚 Documentation Overview

### Getting Started

- **[QUICKSTART.md](../QUICKSTART.md)** ⚡ - Get up and running in 30 minutes
  - Environment setup
  - First build and flash
  - Create your first screen
  - Quick examples

- **[CUSTOMIZATION.md](../CUSTOMIZATION.md)** 📖 - Comprehensive customization guide
  - Firmware architecture overview
  - Development workflow
  - API reference
  - Best practices
  - Common patterns

### In-Depth Guides

- **[ARCHITECTURE.md](ARCHITECTURE.md)** 🏗️ - Deep dive into firmware architecture
  - System architecture
  - Hardware abstraction layer
  - Service architecture
  - Screen management
  - Event system
  - Threading model
  - Memory management
  - Power management
  - Bluetooth architecture

### Tutorials

- **[PONG_GAME.md](tutorials/PONG_GAME.md)** 🎮 - Build a complete Pong game
  - Step-by-step tutorial
  - Motion controls using IMU
  - Game physics
  - Collision detection
  - Threading for smooth gameplay
  - Audio feedback
  - Complete working code

## 🎯 Quick Links

### I want to...

- **Get started quickly** → [QUICKSTART.md](../QUICKSTART.md)
- **Understand the architecture** → [ARCHITECTURE.md](ARCHITECTURE.md)
- **Build a game** → [tutorials/PONG_GAME.md](tutorials/PONG_GAME.md)
- **Learn APIs** → [CUSTOMIZATION.md](../CUSTOMIZATION.md#api-reference)
- **See examples** → Check `main/src/Screens/` directory

### By Topic

#### Hardware

- **Display**: 128x128 ST7735S TFT (see [ARCHITECTURE.md](ARCHITECTURE.md#display))
- **Input**: 4 buttons (see [ARCHITECTURE.md](ARCHITECTURE.md#input))
- **IMU**: LSM6DS3 motion sensor (see [ARCHITECTURE.md](ARCHITECTURE.md#imu))
- **Audio**: Buzzer (see [CUSTOMIZATION.md](../CUSTOMIZATION.md#audio-buzzer))
- **Battery**: Monitoring & management (see [ARCHITECTURE.md](ARCHITECTURE.md#battery))

#### Software

- **Screens**: UI development (see [ARCHITECTURE.md](ARCHITECTURE.md#screen-management))
- **Services**: System services (see [ARCHITECTURE.md](ARCHITECTURE.md#service-architecture))
- **Events**: Event-driven programming (see [ARCHITECTURE.md](ARCHITECTURE.md#event-system))
- **Threading**: FreeRTOS tasks (see [ARCHITECTURE.md](ARCHITECTURE.md#threading-model))
- **Bluetooth**: BLE connectivity (see [ARCHITECTURE.md](ARCHITECTURE.md#bluetooth-architecture))

## 📁 Documentation Structure

```
docs/
├── README.md                    # This file
├── ARCHITECTURE.md              # Detailed architecture guide
└── tutorials/
    └── PONG_GAME.md            # Pong game tutorial

Repository root:
├── QUICKSTART.md               # Quick start guide
├── CUSTOMIZATION.md            # Customization guide
└── README.md                   # Build instructions
```

## 🎓 Learning Path

### Beginner

1. Read [QUICKSTART.md](../QUICKSTART.md)
2. Build and flash the stock firmware
3. Create a simple "Hello World" screen
4. Explore existing screens in `main/src/Screens/`

### Intermediate

1. Read [CUSTOMIZATION.md](../CUSTOMIZATION.md)
2. Use IMU for motion input
3. Add audio feedback
4. Create an interactive application

### Advanced

1. Read [ARCHITECTURE.md](ARCHITECTURE.md)
2. Follow [PONG_GAME.md](tutorials/PONG_GAME.md) tutorial
3. Implement multi-threaded applications
4. Add Bluetooth features
5. Contribute to the firmware

## 💡 Examples in the Repository

### Built-in Applications

1. **Lock Screen** (`main/src/Screens/Lock/`)
   - Watch face
   - Time display
   - Notification icons

2. **Main Menu** (`main/src/Screens/MainMenu/`)
   - Scrollable menu
   - Icon-based navigation
   - Phone connection status

3. **Level Tool** (`main/src/Screens/Level.cpp`)
   - IMU-based bubble level
   - Real-time orientation display
   - Sensor fusion

4. **Theremin** (`main/src/Screens/Theremin/`)
   - Musical instrument
   - IMU-controlled pitch and sequence
   - Audio synthesis

5. **Settings** (`main/src/Screens/Settings/`)
   - Configuration UI
   - Persistent storage
   - Various setting types

6. **Pong Game** (`main/src/Screens/PongGame.cpp`) ⭐ NEW
   - Classic Pong game
   - Motion-controlled paddle
   - Game physics and collision
   - Audio feedback

## 🔧 Development Tools

### Required

- **ESP-IDF 5.1** - Core development framework
- **Git** - Version control
- **C++ Compiler** - Included with ESP-IDF

### Recommended

- **VS Code** with ESP-IDF extension
- **Serial monitor** (minicom, screen, or `idf.py monitor`)
- **GDB** for debugging

## 📖 External Resources

### ESP32 / ESP-IDF

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/)
- [ESP32-S3 Datasheet](https://www.espressif.com/en/products/socs/esp32-s3)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/)

### Graphics

- [LVGL Documentation](https://docs.lvgl.io/)
- [LovyanGFX GitHub](https://github.com/lovyan03/LovyanGFX)

### Language

- [C++ Reference](https://en.cppreference.com/)
- [C++ Best Practices](https://github.com/cpp-best-practices/cppbestpractices)

## 🤝 Contributing

Want to improve the documentation?

1. Fork the repository
2. Make your changes
3. Submit a pull request

Good documentation is:
- Clear and concise
- Has working code examples
- Includes explanations of *why*, not just *how*
- Is well-organized with good headers
- Has proper cross-references

## 📝 Changelog

### v2.0 (Current)
- Added QUICKSTART.md for quick onboarding
- Added CUSTOMIZATION.md with comprehensive guide
- Added ARCHITECTURE.md with deep technical details
- Added Pong game tutorial with complete example
- Added Pong game implementation

### v1.0
- Initial README.md with build instructions

## 📄 License

Documentation is provided as-is. Code examples follow the main repository license.

## 🆘 Getting Help

- **Read the docs first** - Most questions are answered here
- **Check examples** - See how existing code does it
- **Search issues** - Someone may have had the same problem
- **Ask questions** - Create an issue with the "question" label

## ⭐ Highlights

### New to Embedded?

Start with:
1. [QUICKSTART.md](../QUICKSTART.md)
2. ESP-IDF basics
3. Simple screen examples

### Coming from Arduino?

Key differences:
- FreeRTOS tasks instead of loop()
- Event-driven architecture
- More complex build system
- More powerful capabilities

### Game Developer?

Check out:
1. [PONG_GAME.md](tutorials/PONG_GAME.md)
2. IMU for motion controls
3. LVGL for graphics
4. Threading for smooth gameplay

### Building Apps?

Focus on:
1. Screen lifecycle
2. Service pattern
3. Event handling
4. Power management

---

**Happy Coding!** 🚀

If you found this documentation helpful, consider giving the repository a ⭐ star!
