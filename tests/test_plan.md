# Manual test plan

Use a clear area, keep a physical power cutoff available, and lift the wheels for initial motor tests.

1. **Power and Wi-Fi:** verify stable ESP32 startup and address. Repeat with router unavailable; document whether firmware remains blocked.
2. **Motor mapping:** test each channel separately and label physical wheel/direction.
3. **Stop safety:** test explicit stop, host sender termination and Wi-Fi disconnection independently.
4. **LiDAR transport:** compare packet count/checksum errors and observed update rate with the UI's freshness warning.
5. **Scan geometry:** place an obstacle at front/left/right and verify the scan angles.
6. **Mapping:** drive a slow loop; compare the map with the room and note drift.
7. **Navigation:** test open-space goal, goal behind rover, obstacle on route and blocked goal. Record re-plans and stuck recovery.
8. **Stop after goal:** keep recording the map/TF after the rover stops to investigate the reported pose jump.
