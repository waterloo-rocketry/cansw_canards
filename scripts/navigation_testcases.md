Run from the repository root:

```sh
python3 scripts/generate_navigation_testcases.py
```

The script reads the supplied `navigator_log.txt` by default and emits two
headers in `scripts/`. Existing outputs require `--force` to replace them.
You can supply another log as the positional argument and choose paths with
`--output` and `--edited-output`.

- `navigation_testcases.h`: preserves all logged inputs and sensor statuses.
- `navigation_testcases_matlab_edited.h`: enables the four barometer/magnetometer
  statuses in every second record, then gives each enabled sensor an independent
  2% dropout chance in the MATLAB generator's sensor order. Uses Python
  `random.Random(20260716)` by default (`--seed` overrides it); dropout decisions
  are reproducible but differ from MATLAB's exact RNG sequence.

Each header contains seven parallel arrays. The original header uses prefix
`nav_testcases` and count macro `NAV_TESTCASES_COUNT`; the edited header uses
`nav_testcases_matlab_edited` and `NAV_TESTCASES_MATLAB_EDITED_COUNT`.
Both headers can be included in the same translation unit.

| Array suffix | C type per testcase | Navigation argument |
| --- | --- | --- |
| `dt` | `double` | `dt` |
| `flight_phase` | `bool` | `flight_phase` |
| `x` | `double[11]` | `x` |
| `P` | `double[121]` | `P` |
| `bias` | `struct1_T` | `bias` |
| `sens_filt` | `struct2_T` | `sens_filt` |
| `sens_in` | `struct3_T` | `sens_in` |

`P` uses MATLAB column-major indexing, `P[row + 11 * column]`. Struct fields
use the names from `GNC_codegen_types.h`, including the `board_mag_earth` and
`mti_mag_earth` bias fields. MATLAB display scale factors and wrapped matrix
column blocks are supported. Joined four-decimal entries in the supplied log
are split at their four-decimal boundaries. Every record's fields, vector
sizes, matrix shape, and boolean values are validated before output is written.
The header reproduces the log's printed precision; it cannot recover digits
discarded when logging.

The arrays are `static const`, preserving the fixtures and storing an internal
copy in each translation unit that includes them. Include them in your test
harness rather than throughout firmware. Navigation mutates `x`, `P`, `bias`,
and `sens_filt`, so copy those inputs before each independent testcase:

```c
#include "navigation_testcases.h"
#include <string.h>

/* SD must already have valid persistent storage and have been initialized
 * with GNC_codegen_initialize(SD). i must be < NAV_TESTCASES_COUNT. */
void run_navigation_testcase(GNC_codegenStackData *SD, size_t i)
{
    double x[11], P[121];
    memcpy(x, nav_testcases_x[i], sizeof x);
    memcpy(P, nav_testcases_P[i], sizeof P);
    struct1_T bias = nav_testcases_bias[i];
    struct2_T sens_filt = nav_testcases_sens_filt[i];
    double cov_norm, roll_state[2], pdyn;
    bool w_status_nav;
    navigation_codegen_entry(SD, nav_testcases_dt[i],
        nav_testcases_flight_phase[i], x, P, &bias, &sens_filt,
        &nav_testcases_sens_in[i], &cov_norm, roll_state, &pdyn, &w_status_nav);
}
```

Add both `scripts/` and the directory containing `GNC_codegen.h` to the test
compiler's include paths. The headers contain inputs only; they do not define
expected navigation outputs or manage `GNC_codegenStackData`.
