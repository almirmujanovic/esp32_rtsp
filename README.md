# RTSP MJPEG Component for ESP32

A high-performance RTSP server component for ESP32 camera modules, providing real-time MJPEG video streaming over WiFi.

## Features

- RFC 2435 compliant RTP/JPEG streaming
- Optimized for ESP32 WiFi performance
- Support for multiple camera modules (OV2640, OV3660, etc.)
- Automatic DQT table detection
- Flow control and error recovery
- Memory-efficient pre-allocated buffers
- Compatible with VLC, ffplay, and other RTSP clients

## Supported Hardware

- ESP32-CAM
- ESP-EYE
- M5Stack Camera modules
- AI-Thinker modules
- Custom ESP32 + camera sensor boards

## Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/almirmujanovic/esp32-rtsp.git
```

### 2. Configure WiFi
```bash
inside app_main.c
```

### 3. Configure Camera Pins
```bash
idf.py menuconfig
# Navigate to: Component config → Camera configuration
# Select your camera module type
```

### 4. Build and Flash
```bash
idf.py build flash monitor
```

### 5. Connect with Client
```bash
# VLC
vlc rtsp://ESP32_IP:554/track1

# ffplay
ffplay rtsp://ESP32_IP:554/track1
```

## Configuration

### Camera Settings
- Resolution: QVGA to UXGA (depending on module)
- Frame rate: 1-30 FPS (configurable)
- JPEG quality: Adjustable

### Network Settings
- RTSP Port: 554 (default)
- RTP packet size: 1400 bytes (optimized)
- UDP buffer size: 64KB

## API Reference

```c
#include "rtsp_mjpeg.h"

// Start RTSP server
esp_err_t rtsp_mjpeg_server_start(size_t stack_size, UBaseType_t priority);

// Stop RTSP server  
esp_err_t rtsp_mjpeg_server_stop(void);
```

## Performance

- **Latency**: <200ms over local network
- **Throughput**: Up to 2Mbps (depends on resolution/quality)
- **Memory usage**: ~50KB heap + 24KB stack
- **CPU usage**: ~30% at 20 FPS QVGA

## Troubleshooting

### Common Issues

1. **"Camera not found"**
   - Check camera module connection
   - Verify pin configuration in menuconfig

2. **"No video in VLC/ffplay"**
   - Check firewall settings
   - Verify ESP32 IP address
   - Try lower resolution/frame rate

3. **"Connection timeouts"**
   - Check WiFi signal strength
   - Reduce frame rate or resolution
   - Check router UDP buffer settings

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Changelog

### v1.0.0
- Initial release
- RFC 2435 compliant streaming
- Support for major ESP32 camera modules
