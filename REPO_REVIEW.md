# Repository review / publish checklist

- [x] Python/Tkinter application included.
- [x] Setup, architecture, test checklist, and media guidance included.
- [x] ESP32 firmware template included with Wi-Fi credentials separated into a local ignored file.
- [x] `.gitignore` excludes `esp32/secrets.h`, Python caches, ROS build output, and generated maps.
- [ ] Compare the template with the original Arduino IDE sketch; it is not a byte-for-byte copy.
- [ ] Compile the sketch for the exact ESP32-S3 board package.
- [ ] Confirm GPIO 43/44 availability for LiDAR UART on the actual board.
- [ ] Confirm motor direction for each wheel with wheels lifted.
- [ ] Confirm STOP and 500 ms command-timeout behavior.
- [ ] Confirm LiDAR packet checksum and UDP payload compatibility with the Python receiver.
- [ ] Capture real rover/interface/mapping media; do not use mock results as evidence.
- [ ] Remove local credentials/network details and confirm no team-owned/contest code is included.
- [ ] Add a license only if you have the right to license all included code.

## Security note

Never commit `esp32/secrets.h`. The real Wi-Fi credentials previously shared in chat are intentionally not copied into this repository. Rotate the exposed password before public release.
