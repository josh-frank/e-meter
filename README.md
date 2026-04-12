# `e-meter`

## Issues

- **`velocity` in JSON is actually `vel_c` (compressed)** — the field is labeled `"velocity"` but it's sending `vel_c`. Might trip up a consumer of the JSON expecting raw velocity.

- **`WARMUP_SAMP` warmup blocks the splash screen** — `splashScreen()` renders before the warmup loop, but since `delayMicroseconds(PERIOD_US)` is blocking, the display just freezes on "booting..." for 5 seconds.

- **`sntp_sync_rtc` stores local time, not UTC**
```cpp
localtime_r(&now, &t);  // ← converts to TZ_INFO local time
rtc.setDateTime(dt);    // ← stores that in the RTC
```
But `getUnixTime()` in `PCF8563.cpp` treats whatever's in the RTC as UTC when computing the epoch. If your `TZ_INFO` is non-UTC (e.g. `EST5EDT`), the boot anchor printed to Serial will be off by the UTC offset (5–4 hours). You probably want `gmtime_r` here unless you intentionally want the RTC in local time and are correcting for it client-side.

- **`push_poly` uses `millis()` directly, not `t0_ms`-relative time**
The polygram uses wall `millis()` for age, but the JSON frames use `millis() - g_state.t0_ms`. These are consistent for display purposes (only relative age matters for the 30s window), but worth noting they're on different clocks if you ever cross-reference.

- **WebSocket client disconnect detection is lazy**
`ws_send` only marks a slot disconnected when `send` fails. If a client disappears without a proper close frame, the slot stays "connected" until the next send attempt. For a 20 Hz stream this recovers quickly, but for BLE it might be worth the `onDisconnect` callback already in your BLE path.

- **SD SPI pins with expansion board**
`SPI.begin(D8, D9, D10, D2)` hardcodes the Seeed expansion board pinout. Worth a `#elif` guard or comment making clear this will conflict if someone tries a different SPI SD breakout.
