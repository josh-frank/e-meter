# `e-meter`

## Issues

- **`adc_to_uS` called twice per loop** — once inside `next_frame()` (as `uS` in the JSON, using `(int)s.smoothed`) and again in `loop()` (same way). You could return it from `next_frame` or cache it to avoid the redundancy.

- **`compress(g_state.delta)` called twice** — once inside `next_frame` (as `delta_c`) and again in the `renderDisplay` call in `loop()`. The display value isn't using the frame's computed `delta_c` — it's recomputing it. Minor, but inconsistent.

- **`velocity` in JSON is actually `vel_c` (compressed)** — the field is labeled `"velocity"` but it's sending `vel_c`. Might trip up a consumer of the JSON expecting raw velocity.

- **`WARMUP_SAMP` warmup blocks the splash screen** — `splashScreen()` renders before the warmup loop, but since `delayMicroseconds(PERIOD_US)` is blocking, the display just freezes on "booting..." for 5 seconds. Probably fine, just intentional to be aware of.