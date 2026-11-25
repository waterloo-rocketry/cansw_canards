# Canard board firmware module benchmarks

Module runtimes are measured in DWT cycle counters. *Results are cycles at **540 MHz CPU**, not HCLK.*

Clock is CubeMXx `RCC.CpuClockFreq_Value` (`cansw_canard_board_h723.ioc` / `_h733.ioc`). 

## Set up instructions 

Declare `BENCHMARK_FILE_INIT(n, target_period_us)` once at file scope. `n` is the number of slots (indices `0` through `n - 1`); `target_period_us` is a positive compile-time budget in microseconds, shared by all slots in that file. For example, `(2, 1000)` allocates two slots with a 1 ms budget each. The macro converts the budget to cycles using `BENCH_US_TO_CYCLES()`; no separate `BENCH_TARGET_US` definition is required.

```c
BENCHMARK_FILE_INIT(2, 1000);         // file scope

// to benchmark a single module
SINGLE_BENCHMARK_START(0);
do_work();
SINGLE_BENCHMARK_END(0);

// to benchmark multiple modules
SINGLE_BENCHMARK_START(0);
do_work();
do_more_work();
SINGLE_BENCHMARK_END(0);

// nested is also fine 
SINGLE_BENCHMARK_START(0);  
  SINGLE_BENCHMARK_START(1);
  module_a();
  SINGLE_BENCHMARK_END(1);
  module_b();
SINGLE_BENCHMARK_END(0);

benchmark_data_t s;
GET_BENCHMARK_STAT(0, &s);
// mean_cycles = (s.count != 0) ? s.sum / s.count : 0;
BENCHMARK_RESET(0);  // after warmup
```

Note: 
1. Build with `benchmark-` profiles in PIO or the macros compile out. 
2. Call `DWT_init()`. 
3. Benchmark can only run for ~8s before we overflow the `uint32_t` cycle counter. 

## Data collected 

| Field | Meaning |
|---|---|
| `last` `min` `max` | last / best / worst sample, cycles, after warmup |
| `count` `sum` | n and total cycles; **mean = sum / count** |
| `over_budget_count` | samples with `last >=` the file budget in cycles |
| `hist[6]` | counts in % of budget buckets below |

First `BENCH_WARMUP_N` (32) samples per slot are discarded (I/D-cache fill). 

## How to interpret the collected data 

- **mean × loop_Hz / BENCH_CPU_HZ** — fraction of elapsed core cycles covered by the snippet, including interruptions or preemption within the measured interval.
- **max vs budget** — can this miss the period?
- **over_budget_count / count** — how often it did. That is the flight number.
- **hist** (histogram) — one spike vs always slow. Bins are **percent of that file's budget**.

| bin | runtime as percent of budget | meaning                            |
| --- | ----------------------------- | ---------------------------------- |
| 0   | `[0, 10)`                     | cheap                              |
| 1   | `[10, 25)`                    |                                    |
| 2   | `[25, 50)`                    |                                    |
| 3   | `[50, 75)`                    |                                    |
| 4   | `[75, 100)`                   | still made the period              |
| 5   | `[100, inf)`                  | at or above budget — same as `over_budget_count` |

The bin collects the different runtime samples

If bin 5 is a handful and bin 0 is the rest: rare spikes (ISR, SD). If bin 4–5 dominate: the function is eating the period.

## Caution (make sure you read this)

1. To measure each task iteration, put a `START/END` around the execution call.

```c 
// correct example 
void estimator_task(void *arg) {
    for (;;) {
        SINGLE_BENCHMARK_START(0);
        estimator_step();
        SINGLE_BENCHMARK_END(0);
        vTaskDelayUntil(...);
    }
}

// incorrect example
void estimator_task(void *arg) {
    SINGLE_BENCHMARK_START(0);
    for (;;) { ... }
    SINGLE_BENCHMARK_END(0);  // never reached
}
```
2. Beware: all benchmark results are compiled with `-O3` optimizations in place. 
3. The second argument of `BENCHMARK_FILE_INIT(n, target_period_us)` sets the budget for every slot in that `.c` file. The budget classifies samples; it does not enforce a deadline.
4. `START`/`END` are not ISR-safe vs the same index. If you want to time both the ISR and task, assign another time slot to the ISR. 

```c
#include "common/benchmark/benchmark.h"

enum { BENCH_ESTIMATOR = 0, BENCH_MAG_ISR = 1, BENCH_N = 2 };
BENCHMARK_FILE_INIT(BENCH_N, 1000);  // 1 ms budget for both slots

void estimator_task(void *arg) {
    for (;;) {
        SINGLE_BENCHMARK_START(BENCH_ESTIMATOR);
        estimator_step();
        SINGLE_BENCHMARK_END(BENCH_ESTIMATOR);
        vTaskDelayUntil(...);
    }
}

void EXTI9_5_IRQHandler(void) {
    SINGLE_BENCHMARK_START(BENCH_MAG_ISR);
    HAL_GPIO_EXTI_IRQHandler(IMU_INT1_Pin);
    SINGLE_BENCHMARK_END(BENCH_MAG_ISR);
}
```

5. (I repeat) This benchmark discards the first 32 samples as warmup. They will not be included in the reported data. 

