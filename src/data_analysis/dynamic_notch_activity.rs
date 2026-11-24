// src/data_analysis/dynamic_notch_activity.rs

use crate::axis_names::AXIS_NAMES;
use crate::constants::{
    DYN_NOTCH_ACTIVITY_HIGH_PERCENTILE_PCT, DYN_NOTCH_ACTIVITY_LOW_PERCENTILE_PCT,
    DYN_NOTCH_ACTIVITY_MAX_NOTCHES, FFT_FREQ_CENTRE_COLUMNS_CENTRES_FIRST,
    FFT_FREQ_CENTRE_COLUMNS_GYRO_FIRST, PARITY_DIVISOR,
};
use crate::data_input::log_data::LogRowData;

/// Which `debug[]` columns carry the notch centre frequencies when `debug_mode` is FFT_FREQ.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FftFreqLayout {
    /// Betaflight 4.5 and older, EmuFlight: centres in `debug[0..2]`, pre-notch gyro in `debug[3]`.
    CentresFirst,
    /// Betaflight 4.6+ / 2025.12+: pre-notch gyro in `debug[0]`, centres in `debug[1..3]`.
    GyroFirst,
}

impl FftFreqLayout {
    /// `debug[]` index of each notch centre, notch 1 first.
    pub fn centre_columns(self) -> [usize; DYN_NOTCH_ACTIVITY_MAX_NOTCHES] {
        match self {
            FftFreqLayout::CentresFirst => FFT_FREQ_CENTRE_COLUMNS_CENTRES_FIRST,
            FftFreqLayout::GyroFirst => FFT_FREQ_CENTRE_COLUMNS_GYRO_FIRST,
        }
    }
}

/// Centre frequency of one notch over time.
#[derive(Debug, Clone)]
pub struct NotchActivity {
    /// Notch number, 1-based.
    pub notch: usize,
    /// Runs of consecutive rows with a centre above 0: `(time s, centre Hz)`. A row without a
    /// centre ends the run, so a gap is never drawn as a line.
    pub segments: Vec<Vec<(f64, f64)>>,
    /// Rows with a time stamp and a finite value in this column.
    pub samples: usize,
    /// Rows whose value is 0 or below: the firmware had no centre for this notch.
    pub no_peak_samples: usize,
    pub min_hz: f64,
    pub max_hz: f64,
    pub median_hz: f64,
    /// Lower edge of the centre spread (`DYN_NOTCH_ACTIVITY_LOW_PERCENTILE_PCT`).
    pub low_hz: f64,
    /// Upper edge of the centre spread (`DYN_NOTCH_ACTIVITY_HIGH_PERCENTILE_PCT`).
    pub high_hz: f64,
}

impl NotchActivity {
    /// Width of the central spread of the centre frequency, Hz.
    pub fn span_hz(&self) -> f64 {
        self.high_hz - self.low_hz
    }
}

/// Dynamic notch centres of the gyro debug axis.
#[derive(Debug, Clone)]
pub struct DynamicNotchActivity {
    /// Axis index from the `gyro_debug_axis` header; `None` when the header has none (Betaflight 4.5
    /// and older, EmuFlight).
    pub debug_axis: Option<usize>,
    /// `(dyn_notch_min_hz, dyn_notch_max_hz)` from the header, when a dynamic notch is configured.
    pub range_hz: Option<(f64, f64)>,
    /// Notches with at least one centre above 0, notch 1 first.
    pub notches: Vec<NotchActivity>,
}

/// Why no activity was produced; the text goes to the console as a one-line note.
pub const SKIP_NOTE_NOT_FFT_FREQ: &str =
    "Dynamic Notch Activity skipped: debug_mode is not FFT_FREQ, or the FFT_FREQ layout or firmware version (Betaflight before 4.3, EmuFlight before 0.4) is not supported.";
pub const SKIP_NOTE_NO_CENTRES: &str =
    "Dynamic Notch Activity skipped: no notch centre frequency above 0 in the FFT_FREQ debug columns.";

/// Value at percentile `pct` of ascending `sorted`, index `(n - 1) * pct / 100` (integer
/// division). The motor-desync and stick-distribution percentiles are private to their modules
/// and use other index rules; this one needs the exact index for whole-number percentiles.
fn percentile_of_sorted(sorted: &[f64], pct: usize) -> f64 {
    let idx = (sorted.len() - 1) * pct.min(100) / 100;
    sorted[idx]
}

/// Median of ascending `sorted` (mean of the two middle values for an even count).
fn median_of_sorted(sorted: &[f64]) -> f64 {
    let mid = sorted.len() / 2;
    if sorted.len().is_multiple_of(PARITY_DIVISOR) {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

/// Builds one notch's series from the rows. `None` when no row has a centre above 0.
fn build_notch(notch: usize, column: usize, rows: &[LogRowData]) -> Option<NotchActivity> {
    let mut segments: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut run: Vec<(f64, f64)> = Vec::new();
    let mut centres: Vec<f64> = Vec::new();
    let mut samples = 0usize;
    let mut no_peak_samples = 0usize;

    for row in rows {
        let (Some(time), Some(value)) = (row.time_sec, row.debug.get(column).copied().flatten())
        else {
            if !run.is_empty() {
                segments.push(std::mem::take(&mut run));
            }
            continue;
        };
        if !time.is_finite() || !value.is_finite() {
            if !run.is_empty() {
                segments.push(std::mem::take(&mut run));
            }
            continue;
        }
        samples += 1;
        if value > 0.0 {
            run.push((time, value));
            centres.push(value);
        } else {
            no_peak_samples += 1;
            if !run.is_empty() {
                segments.push(std::mem::take(&mut run));
            }
        }
    }
    if !run.is_empty() {
        segments.push(run);
    }
    if centres.is_empty() {
        return None;
    }
    centres.sort_by(|a, b| a.total_cmp(b));
    Some(NotchActivity {
        notch,
        segments,
        samples,
        no_peak_samples,
        min_hz: centres[0],
        max_hz: centres[centres.len() - 1],
        median_hz: median_of_sorted(&centres),
        low_hz: percentile_of_sorted(&centres, DYN_NOTCH_ACTIVITY_LOW_PERCENTILE_PCT),
        high_hz: percentile_of_sorted(&centres, DYN_NOTCH_ACTIVITY_HIGH_PERCENTILE_PCT),
    })
}

/// Axis index from the `gyro_debug_axis` header, when present and a known axis.
fn debug_axis_from_header(header_metadata: &[(String, String)]) -> Option<usize> {
    header_metadata
        .iter()
        .find(|(k, _)| k == "gyro_debug_axis")
        .and_then(|(_, v)| v.trim().parse::<usize>().ok())
        .filter(|&axis| axis < AXIS_NAMES.len())
}

/// Notch centre frequencies over time. `layout` comes from the header (`debug_mode` and firmware
/// revision, see `debug_mode_lookup::fft_freq_layout`). `None` when no notch has a centre above 0.
pub fn analyze_dynamic_notch_activity(
    rows: &[LogRowData],
    header_metadata: &[(String, String)],
    layout: FftFreqLayout,
    range_hz: Option<(f64, f64)>,
) -> Option<DynamicNotchActivity> {
    let notches: Vec<NotchActivity> = layout
        .centre_columns()
        .iter()
        .enumerate()
        .filter_map(|(i, &column)| build_notch(i + 1, column, rows))
        .collect();
    if notches.is_empty() {
        return None;
    }
    Some(DynamicNotchActivity {
        debug_axis: debug_axis_from_header(header_metadata),
        range_hz,
        notches,
    })
}

/// One frequency as shown in every output (Hz, no decimals).
pub fn format_hz(hz: f64) -> String {
    format!("{hz:.0}")
}

/// Name of the logged axis for titles, console and report.
pub fn debug_axis_text(activity: &DynamicNotchActivity) -> String {
    match activity.debug_axis {
        Some(axis) => format!("{} axis", AXIS_NAMES[axis]),
        None => "gyro debug axis".to_string(),
    }
}

/// `median 203 Hz, P5-P95 153-339 Hz (span 186 Hz), min-max 140-511 Hz`: the single text for
/// console and report.
pub fn centre_summary_text(n: &NotchActivity) -> String {
    format!(
        "median {} Hz, P{}-P{} {}-{} Hz (span {} Hz), min-max {}-{} Hz",
        format_hz(n.median_hz),
        DYN_NOTCH_ACTIVITY_LOW_PERCENTILE_PCT,
        DYN_NOTCH_ACTIVITY_HIGH_PERCENTILE_PCT,
        format_hz(n.low_hz),
        format_hz(n.high_hz),
        format_hz(n.span_hz()),
        format_hz(n.min_hz),
        format_hz(n.max_hz),
    )
}

/// Plot legend entry of one notch.
pub fn legend_text(n: &NotchActivity) -> String {
    format!("Notch {} (median {} Hz)", n.notch, format_hz(n.median_hz))
}

/// Console lines, one per notch.
pub fn console_lines(activity: &DynamicNotchActivity) -> Vec<String> {
    activity
        .notches
        .iter()
        .map(|n| {
            format!(
                "  Notch {} ({}, {} samples, {} no peak): {}",
                n.notch,
                debug_axis_text(activity),
                n.samples,
                n.no_peak_samples,
                centre_summary_text(n)
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // Header sets as the parser returns them, from real logs of each firmware generation.
    fn header(fw: &str, mode: &str, extra: &[(&str, &str)]) -> Vec<(String, String)> {
        let mut h = vec![
            ("Firmware revision".to_string(), fw.to_string()),
            ("debug_mode".to_string(), mode.to_string()),
        ];
        h.extend(extra.iter().map(|(k, v)| (k.to_string(), v.to_string())));
        h
    }

    fn bf45_header() -> Vec<(String, String)> {
        header(
            "Betaflight 4.5.1 (77d01ba3b) STM32H743",
            "17",
            &[("dyn_notch_count", "2")],
        )
    }

    fn bf2025_header() -> Vec<(String, String)> {
        header(
            "Betaflight 2025.12.2 (79065c96b) STM32F7X2",
            "16",
            &[("dyn_notch_count", "1"), ("gyro_debug_axis", "0")],
        )
    }

    fn row(t: f64, debug: [Option<f64>; 4]) -> LogRowData {
        LogRowData {
            time_sec: Some(t),
            debug,
            ..Default::default()
        }
    }

    // 4.5 layout (count 2): debug[0], debug[1] centres, debug[2] unused slot (0), debug[3] gyro.
    fn bf45_rows(centres: &[(f64, f64)]) -> Vec<LogRowData> {
        centres
            .iter()
            .enumerate()
            .map(|(i, (c1, c2))| {
                row(
                    i as f64,
                    [Some(*c1), Some(*c2), Some(0.0), Some(-300.0 + i as f64)],
                )
            })
            .collect()
    }

    // 2025.12 layout (count 1): debug[0] gyro, debug[1] centre, debug[2..3] unused slots (0).
    fn bf2025_rows(centres: &[f64]) -> Vec<LogRowData> {
        centres
            .iter()
            .enumerate()
            .map(|(i, c)| {
                row(
                    i as f64,
                    [Some(-5.0 + i as f64), Some(*c), Some(0.0), Some(0.0)],
                )
            })
            .collect()
    }

    #[test]
    fn column_map_differs_between_layouts() {
        assert_eq!(FftFreqLayout::CentresFirst.centre_columns(), [0, 1, 2]);
        assert_eq!(FftFreqLayout::GyroFirst.centre_columns(), [1, 2, 3]);
    }

    // The pre-notch gyro column must never be read as a centre: a signed value near 0 would show as
    // "no peak", and a large one as a centre.
    #[test]
    fn legacy_layout_reads_two_centres_and_skips_gyro_and_unused_slot() {
        let rows = bf45_rows(&[(200.0, 400.0), (210.0, 410.0), (220.0, 420.0)]);
        let a = analyze_dynamic_notch_activity(
            &rows,
            &bf45_header(),
            FftFreqLayout::CentresFirst,
            Some((125.0, 650.0)),
        )
        .expect("activity");
        assert_eq!(a.notches.len(), 2);
        assert_eq!(a.notches[0].notch, 1);
        assert_eq!(a.notches[0].min_hz, 200.0);
        assert_eq!(a.notches[0].max_hz, 220.0);
        assert_eq!(a.notches[1].min_hz, 400.0);
        assert_eq!(a.debug_axis, None);
        assert_eq!(a.range_hz, Some((125.0, 650.0)));
    }

    #[test]
    fn new_layout_reads_one_centre_from_column_1_not_the_gyro_in_column_0() {
        let rows = bf2025_rows(&[150.0, 160.0, 170.0]);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        assert_eq!(a.notches.len(), 1);
        assert_eq!(a.notches[0].min_hz, 150.0);
        assert_eq!(a.notches[0].max_hz, 170.0);
        assert_eq!(a.debug_axis, Some(0));
        assert_eq!(debug_axis_text(&a), "Roll axis");
    }

    // The same rows read with the wrong layout give different centres, so the layout matters.
    #[test]
    fn wrong_layout_reads_different_columns() {
        let rows = bf2025_rows(&[150.0, 160.0]);
        let a = analyze_dynamic_notch_activity(
            &rows,
            &bf2025_header(),
            FftFreqLayout::CentresFirst,
            None,
        )
        .expect("activity");
        // debug[0] is the gyro (-5, -4): never above 0, so only column 1 (150, 160) qualifies,
        // and it is notch 2 of this layout.
        assert_eq!(a.notches.len(), 1);
        assert_eq!(a.notches[0].notch, 2);
    }

    // Spec: centre 0 is "no peak": counted as a sample, never drawn, never in the statistics,
    // and it splits the line.
    #[test]
    fn zero_centre_is_no_peak_and_splits_the_line() {
        let rows = bf2025_rows(&[100.0, 0.0, 200.0, 300.0, 0.0]);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        let n = &a.notches[0];
        assert_eq!(n.samples, 5);
        assert_eq!(n.no_peak_samples, 2);
        assert_eq!(n.segments.len(), 2);
        assert_eq!(n.segments[0], [(0.0, 100.0)]);
        assert_eq!(n.segments[1], [(2.0, 200.0), (3.0, 300.0)]);
        assert_eq!(n.min_hz, 100.0);
        assert_eq!(n.max_hz, 300.0);
        assert_eq!(n.median_hz, 200.0);
    }

    #[test]
    fn all_zero_centres_give_no_activity() {
        let rows = bf2025_rows(&[0.0, 0.0, 0.0]);
        assert!(analyze_dynamic_notch_activity(
            &rows,
            &bf2025_header(),
            FftFreqLayout::GyroFirst,
            None
        )
        .is_none());
    }

    #[test]
    fn no_rows_give_no_activity() {
        assert!(analyze_dynamic_notch_activity(
            &[],
            &bf2025_header(),
            FftFreqLayout::GyroFirst,
            None
        )
        .is_none());
    }

    // Spec: one sample gives median = min = max = every percentile, span 0.
    #[test]
    fn single_sample_has_zero_span() {
        let rows = bf2025_rows(&[321.0]);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        let n = &a.notches[0];
        assert_eq!(n.samples, 1);
        assert_eq!(
            (n.min_hz, n.max_hz, n.median_hz, n.low_hz, n.high_hz),
            (321.0, 321.0, 321.0, 321.0, 321.0)
        );
        assert_eq!(n.span_hz(), 0.0);
        assert_eq!(n.segments, vec![vec![(0.0, 321.0)]]);
    }

    // Spec: centres 1..=101 (n = 101): P5 index 100*5/100 = 5 -> 6, P95 index 95 -> 96, median 51.
    #[test]
    fn spread_follows_the_percentile_definition() {
        let centres: Vec<f64> = (1..=101).map(f64::from).collect();
        let rows = bf2025_rows(&centres);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        let n = &a.notches[0];
        assert_eq!((n.low_hz, n.median_hz, n.high_hz), (6.0, 51.0, 96.0));
        assert_eq!(n.span_hz(), 90.0);
    }

    #[test]
    fn even_count_median_is_the_midpoint() {
        let rows = bf2025_rows(&[100.0, 200.0, 300.0, 400.0]);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        assert_eq!(a.notches[0].median_hz, 250.0);
    }

    // A row without a time or with a NaN value ends the run and is not a sample.
    #[test]
    fn missing_time_or_non_finite_value_breaks_the_run() {
        let mut rows = bf2025_rows(&[100.0, 110.0, 120.0, 130.0]);
        rows[1].time_sec = None;
        rows[2].debug[1] = Some(f64::NAN);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        let n = &a.notches[0];
        assert_eq!(n.samples, 2);
        assert_eq!(n.segments.len(), 2);
    }

    // A header without the debug column leaves `debug[i]` as None in every row.
    #[test]
    fn absent_debug_columns_give_no_activity() {
        let rows: Vec<LogRowData> = (0..5).map(|i| row(f64::from(i), [None; 4])).collect();
        assert!(analyze_dynamic_notch_activity(
            &rows,
            &bf2025_header(),
            FftFreqLayout::GyroFirst,
            None
        )
        .is_none());
    }

    #[test]
    fn debug_axis_header_is_validated() {
        let h = |v: &str| {
            header(
                "Betaflight 2025.12.2 (x) F7",
                "16",
                &[("gyro_debug_axis", v)],
            )
        };
        assert_eq!(debug_axis_from_header(&h("2")), Some(2));
        assert_eq!(debug_axis_from_header(&h("3")), None);
        assert_eq!(debug_axis_from_header(&h("x")), None);
        assert_eq!(debug_axis_from_header(&bf45_header()), None);
    }

    #[test]
    fn console_and_legend_share_the_formatted_values() {
        let rows = bf2025_rows(&[100.0, 200.0, 300.0]);
        let a =
            analyze_dynamic_notch_activity(&rows, &bf2025_header(), FftFreqLayout::GyroFirst, None)
                .expect("activity");
        let n = &a.notches[0];
        assert_eq!(
            centre_summary_text(n),
            "median 200 Hz, P5-P95 100-200 Hz (span 100 Hz), min-max 100-300 Hz"
        );
        assert_eq!(legend_text(n), "Notch 1 (median 200 Hz)");
        let lines = console_lines(&a);
        assert_eq!(lines.len(), 1);
        assert!(lines[0].contains(&centre_summary_text(n)), "{}", lines[0]);
        assert!(lines[0].contains("Roll axis"), "{}", lines[0]);
    }
}
