---
description: Switch to a branch, merge main (recursing submodules), and update that module's outdated health-check error codes to match canlib/message_types.h
argument-hint: <branch-name> <module-name>
---

Arguments: `$1` = branch name to check out, `$2` = module name (matches the driver folder/file under `src/drivers/<module>/` and the enum suffix in canlib, e.g. `IIS2MDC`, `ADXL380`, `sd_card`).

Do the following, in order:

## 1. Switch and merge
- `git status` first; if the working tree is dirty, stop and ask before doing anything destructive.
- `git fetch origin`
- `git checkout $1` (if it doesn't exist locally, check out `origin/$1`)
- `git merge main`
- If there are conflicts:
  - For submodule conflicts: check `git merge-base --is-ancestor <branch-commit> <main-commit>` inside the submodule. If main's commit is a descendant, just check out main's commit in the submodule and `git add` it. If it's NOT a simple fast-forward, stop and ask.
  - For other file conflicts (e.g. `health_checks.c`): resolve by taking main's version of any renamed/refactored identifiers (like enum renames), while preserving whatever this branch was adding (e.g. a new module's status function wired into the array). Show me the resolved diff before committing if it's not a trivial rename.
  - Commit the merge once resolved.
- `git submodule update --init --recursive`
- Confirm `git status` is clean after.

## 2. Fix outdated health-check error codes for `$2`
- In `src/drivers/$2/$2.c` (adjust path/casing as needed — check actual file name), find the `*_get_status()` function.
- Identify any old-style identifiers that don't match current naming: `.module_id = MODULE_<X>` and `status.error_bitfield |= 1 << ERR_<REASON>`.
- Cross-reference `src/third_party/canlib/message_types.h` for the current names:
  - Module ID enum is `CANARDS_MODULE_ID_<X>`
  - Error bit offsets are `CANARDS_MODULE_E_<REASON>_OFFSET`
- Before renaming, verify the mapping is an unambiguous 1:1 by grepping how other already-migrated drivers (e.g. `movella.c`, `LSM6DSV32X.c`, `sd_card.c`) use the same `CANARDS_MODULE_E_*_OFFSET` constants in the same pattern. If a clean match isn't obvious, stop and ask rather than guess.
- Apply the rename in `$2.c` (and `.h` if it also references old identifiers).
- Re-grep the driver's `.c`/`.h` files for any remaining stale `MODULE_<X>` / `ERR_<X>` references to confirm nothing was missed.
- Do NOT touch unrelated modules' stale references even if spotted in passing (e.g. in test files) — just mention them, don't fix them.
- Commit this as a separate commit from the merge commit.

## 3. Report
At the end, summarize what was done and list every git command actually run (checkout, merge, submodule update, commits, any conflict-resolution commands).
