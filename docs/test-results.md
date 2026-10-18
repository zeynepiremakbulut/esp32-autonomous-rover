# Test results

| Test | Procedure | Result | Evidence / notes |
|---|---|---|---|
| ESP32 Wi-Fi connection | Confirm board joins expected network and IP is reachable | Not recorded | |
| Motor directions | Lift wheels; test each drive channel individually | Not recorded | |
| Stop command | Send `M:x` and confirm motors stop | Not recorded | |
| 500 ms watchdog | Stop sending motor packets during movement test | Not recorded | |
| Wi-Fi loss | Interrupt network while wheels are lifted | Not recorded | |
| LiDAR packet stream | Verify packets and checksum behavior | Not recorded | |
| `/scan` | Check angle/range values and update rate | Not recorded | |
| Radar orientation | Compare a known obstacle to physical direction | Not recorded | |
| Cartographer map | Walk/drive a loop and inspect closure/drift | Not recorded | |
| Goal navigation | Run several goals in a clear, static space | Not recorded | |
| Stop-at-goal localization | Observe TF/map pose after stopping | Not recorded | |
| Link-loss stop in Python | Interrupt LiDAR/host link and check stop behavior | Not recorded | |
