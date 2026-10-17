# ESP32 firmware

`rover_firmware.ino` is the firmware source shared by the project owner. The only source-code edits made for this repository copy are replacing the Wi-Fi SSID and password literals with `write-your-own-ssid` and `write-your-own-password`. No motor logic, pins, packet handling, timing, or other code was intentionally changed.

## Before uploading

1. Replace `write-your-own-ssid` and `write-your-own-password` locally with your Wi-Fi credentials. Do not commit real credentials.
2. The sketch uses a static ESP32 IP `192.168.1.3`, gateway `192.168.1.1`, subnet `255.255.255.0`; confirm these match your network.
3. Check that the Python PC address/network and UDP ports match: motor `4210`, LiDAR `4211`.
4. With wheels lifted, test motor direction, `M:x`, the 500 ms command watchdog and Wi-Fi loss before driving on the floor.
5. Confirm GPIO 43/44 use for LiDAR UART does not conflict with your board's serial/debug setup.

## Protocol details from the sketch

- Motor commands: `V:<left>,<right>`, `M:w`, `M:s`, `M:a`, `M:d`, `M:x`
- Motor watchdog: 500 ms
- LiDAR UART: 115200 baud, RX GPIO 44, TX GPIO 43
- LiDAR data: 22-byte packets, `0xFA` start byte plus index byte, checksum validation, batches of up to 5 packets and 40 ms flush interval
