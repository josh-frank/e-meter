# `e-meter`

## Issues

- **`velocity` in JSON is actually `vel_c` (compressed)** — the field is labeled `"velocity"` but it's sending `vel_c`. Might trip up a consumer of the JSON expecting raw velocity.

- **`WARMUP_SAMP` warmup blocks the splash screen** — `splashScreen()` renders before the warmup loop, but since `delayMicroseconds(PERIOD_US)` is blocking, the display just freezes on "booting..." for 5 seconds.

- **`push_poly` uses `millis()` directly, not `t0_ms`-relative time**
The polygram uses wall `millis()` for age, but the JSON frames use `millis() - g_state.t0_ms`. These are consistent for display purposes (only relative age matters for the 30s window), but worth noting they're on different clocks if you ever cross-reference.

- **WebSocket client disconnect detection is lazy**
`ws_send` only marks a slot disconnected when `send` fails. If a client disappears without a proper close frame, the slot stays "connected" until the next send attempt. For a 20 Hz stream this recovers quickly, but for BLE it might be worth the `onDisconnect` callback already in your BLE path.

- **No watchdog feed**
At 20 Hz with WiFi enabled, the `ws_accept()` + `ws_send()` calls can occasionally stall long enough to trigger the ESP32's task watchdog. Add `esp_task_wdt_reset()` at the top of `loop()` (include `<esp_task_wdt.h>`).

- **SD SPI pins with expansion board**
`SPI.begin(D8, D9, D10, D2)` hardcodes the Seeed expansion board pinout. Worth a `#elif` guard or comment making clear this will conflict if someone tries a different SPI SD breakout.

- **ADC nonlinearity on ESP32S3**
The ESP32's ADC is notoriously nonlinear, especially near the rails. You're clamping to `1..4094` which helps, but consider adding a simple two-point or polynomial calibration. Seeed publishes correction curves for the XIAO S3. Alternatively, use the IDF's `esp_adc_cal` (available via `esp_adc/adc_coneshot.h`) which gives you ~±2% linearity vs ~±10% uncorrected.

- **The `compress()` function**
`x / (1 + 0.05|x|)` is a reasonable soft-limiter but its knee is at x=20 which is quite wide. For the "needle deflection" feel of a real e-meter, you might want a steeper compression — the original Mark V used a roughly logarithmic response. Consider `sign(x) * log(1 + k*|x|) / log(1 + k*100)` with k tunable.

- **Missing: tone arm equivalent**
The real e-meter has two controls: **sensitivity** (gain) and **tone arm** (baseline offset/null control). You have sensitivity baked into the EMA ratio but no user-adjustable tone arm to "set the needle to the middle." For the physical prop experience, this is the most important missing feature — without it the needle drifts off-scale and has to be re-nulled by the operator constantly. You could implement a software tone arm as a manually-triggered baseline reset, or a dial input via a potentiometer on a second ADC pin.

- **JSON frame is missing `count`**
`s.count` is incremented but never included in the JSON output. Useful for detecting dropped frames on the receiver side — worth adding.
