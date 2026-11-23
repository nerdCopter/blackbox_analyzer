## Scope
- This repo holds math, physics, logs and the data-driven application only.
- Never put aircraft, build, pilot or owner references, log or file names from a specific set, log counts, or personal and workflow remarks in code, comments, docs, tests, commit messages or PR text.
- Calibration constants say only what is empirical and how to revisit it; the evidence from specific logs stays outside this repo.

## Code Changes
- Preserve comments unless related code changes
- Add only functional comments, no AI/history notes
- Ask before removing any debug console output
- **All constants go in `src/constants.rs`** — no hardcoded magic numbers in function code
  - **Exception:** Axis indices (0=Roll, 1=Pitch, 2=Yaw) are managed via `src/axis_names.rs` module with `AXIS_COUNT`, `AXIS_NAMES`, and `axis_name()` function. Use these instead of creating redundant axis index constants.
  - See `src/axis_names.rs` for centralized axis naming (commit 73f8c04)
- **Before implementing a new statistical/analysis computation**, grep `src/data_analysis/` and `src/plot_functions/` for an existing implementation of the same core primitive (cross-correlation, percentile thresholding, per-row error, event/threshold-crossing counting). Reuse or extend it; if a genuinely new computation is needed, note in a code comment why the existing ones don't cover this case.
- **Output parity (console, report.md, PNG titles and legends):** any value, label or recommendation shown in more than one output must come from one shared function, never from a copy in each output.
  - All outputs use the same values and the same decision rules; only presentation differs.
  - Pass data the report needs through the result structs (`StepAxisReport`, `FlightReport`); do not re-derive it in `report.rs`.
  - Console and report may be more verbose (extra fields, thresholds, reasoning). Plot titles and legends stay minimal and concise.
  - Changing one output updates the others in the same change, with a test that the report carries the shared function's text.
- Run checks in this order:
  1) `cargo clippy --all-targets --all-features -- -D warnings` — fix all warnings.
  2) `cargo fmt --all` — only after clippy passes.

## Firmware Detection
- **ALWAYS** use `Firmware revision` header to detect firmware type
- **NEVER** use `Firmware type` (unreliable - Emuflight reports as "Cleanflight")
- Firmware revision patterns:
  - Betaflight: `"Betaflight 4.x.x"` or `"Betaflight YYYY.MM"` or similar
  - Emuflight: `"EmuFlight 0.x.x"` or similar  
  - INAV: `"INAV x.x.x"` or similar

## Testing
- **ALWAYS** prefer `cargo build --release` rather than `cargo run`
- **ALWAYS** use `--output-dir` ./output/
- Temporary test code must go in ./tests/ and is deleted before committing
- Run the full `cargo test` and read every `test result` line. The pre-commit hook runs clippy and fmt only, so integration tests in `tests/` can break unseen.
- Disk-writing unit tests **must** use a unique path per test (e.g. a per-call temp subdirectory) — `cargo test` runs concurrently by default, and a shared path races

## Git Workflow  
- **NEVER** use `git add -A` or `git add .`
- **NEVER** automatically commit changes without an explicit user request
- Only stage modified files and specified files when commanded
- Update `.gitignore` when required
- **ONLY** commit when explicitly commanded with phrases such as:
  - "commit"
  - "git commit"
  - "commit changes" 
  - "git add and commit"
  - "stage and commit"
- Generate concise commit messages from `git diff HEAD`

## Development Setup
- Use `.github/setup-dev.sh` for environment setup
- Pre-commit hook enforces formatting and clippy checks
- All changes must pass CI pipeline
## Documentation
- **OVERVIEW.md Maintenance:** For new features, add documentation in `OVERVIEW.md` at proper, appropriate locations
  - Information should be balanced or concise; never overly verbose
  - Align verbosity with existing documentation style in OVERVIEW.md
  - Verify placement fits the document structure and flow
- Update Table of Contents in OVERVIEW.md when adding new sections
- Keep README.md usage/examples synchronized with actual CLI flags in `src/main.rs`
  - The README usage block holds the `--help` text verbatim (only the binary name differs); the test `readme_usage_block_matches_help_text` fails when they differ. Change `usage_text` in `src/main.rs` and the README together.
- The Optimal P recommendation table in `OVERVIEW.md` is pinned by the test `recommendation_table_follows_td_zone_and_noise`. Change the rule, the table and the test together.