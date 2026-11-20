# Benchmark

DWT cycle counter around a snippet. One slot = one owner (one task or ISR). Results are cycles at **540 MHz CPU**, not HCLK.

Clock is CubeMX `RCC.CpuClockFreq_Value` (`cansw_canard_board_h723.ioc` / `_h733.ioc`). `BENCH_CPU_HZ` must match it. `SystemCoreClock` is 270 MHz here — do not divide by that.

```
us     = cycles / BENCH_CYCLES_PER_US     # 540
mean   = sum / count                      # integer, after warmup
budget = BENCH_US_TO_CYCLES(BENCH_TARGET_US)
```

Set `BENCH_TARGET_US` to the period you must beat, in microseconds (10000 = 10 ms / 100 Hz, 2500 = 2.5 ms / 400 Hz nav, 1000 = 1 ms).

## Use

```c
#define BENCH_TARGET_US 10000u  // 10 ms
BENCHMARK_FILE_INIT(2);         // file scope

SINGLE_BENCHMARK_START(0);
do_work();
SINGLE_BENCHMARK_END(0);

benchmark_data_t s;
GET_BENCHMARK_STAT(0, &s);
// mean_cycles = (s.count != 0) ? s.sum / s.count : 0;
BENCHMARK_RESET(0);  // after warmup soak, or on flight-phase change
```

Build with `-DENABLE_BENCHMARK` or the macros compile out. Call `DWT_init()` once in `main`. Window must be shorter than one CYCCNT wrap (~8 s).

## What is stored

| Field | Meaning |
|---|---|
| `last` `min` `max` | last / best / worst sample, cycles, after warmup |
| `count` `sum` | n and total cycles; **mean = sum / count** |
| `over_budget_count` | samples with `last >` the file budget |
| `hist[6]` | counts in % of budget buckets below |

First `BENCH_WARMUP_N` (32) samples per slot are discarded (I/D-cache fill). No float. No on-chip P50/P99.

## How to read it

- **mean × loop_Hz / CPU_HZ** — fraction of the core that snippet uses.
- **max vs budget** — can this miss the period?
- **over_budget_count / count** — how often it did. That is the flight number.
- **hist** — one spike vs always slow. Bins are **percent of that file's budget**, so 1 kHz and 100 Hz use the same chart. Graph the six bins; do not invent P99 from them.

| bin | percentile runtime WRT budget | meaning                            |
| --- | ----------------------------- | ---------------------------------- |
| 0   | `[0, 10)`                     | cheap                              |
| 1   | `[10, 25)`                    |                                    |
| 2   | `[25, 50)`                    |                                    |
| 3   | `[50, 75)`                    |                                    |
| 4   | `[75, 100)`                   | still made the period              |
| 5   | `[100, inf)`                  | miss — same as `over_budget_count` |

```
i   % budget         example (mass on the left = healthy)
0   [  0,  10)       ############  12000
1   [ 10,  25)       #####          5100
2   [ 25,  50)       ##             1200
3   [ 50,  75)       #               100
4   [ 75, 100)                       29
5   [100,  inf)                       3   <- misses
```

`benchmark_hist_ascii(s.hist, buf, n)` prints that. Coarse percentile: walk bins until cumulative count crosses 50% / 90% of `count` — you get a **range** (e.g. P90 in 25–50% of budget), not 187 µs. Real Pxx needs a sample dump off-chip.

If bin 5 is a handful and bin 0 is the rest: rare spikes (ISR, SD). If bin 4–5 dominate: the function is eating the period.

## Rules

1. Probe only under `ENABLE_BENCHMARK`. Report git hash, `-O3`, caches on/off with the number.
2. One slot, one context. `START`/`END` are not ISR-safe vs the same index.
3. Budget = `BENCH_TARGET_US` for that file. One target per `.c`.
4. Skip 32, then count. `BENCHMARK_RESET` on phase change (pad vs boost is a different workload).
5. Publish last / min / max / count / sum / over_budget_count / hist. Mean at print time. Histogram graph next to the counts. No float average, no MCU P99.
6. TIM23 is FreeRTOS runtime stats (1 MHz), not this probe. Do not fail flight timebase init if TIM23 fails.
