// src/data_analysis/vibration_percentile.rs

use crate::axis_names::AXIS_NAMES;
use crate::constants::{VIBRATION_ECDF_MAX_POINTS, VIBRATION_PERCENTILES_PCT};
use crate::data_input::log_data::LogRowData;

/// Number of reported percentiles (`VIBRATION_PERCENTILES_PCT`).
const PERCENTILE_COUNT: usize = VIBRATION_PERCENTILES_PCT.len();

/// Distribution of |gyro| for one axis and one signal (filtered or unfiltered).
pub struct VibrationSeries {
    /// Finite samples used.
    pub count: usize,
    /// Values at `VIBRATION_PERCENTILES_PCT`, in the same order, deg/s.
    pub percentiles: [f64; PERCENTILE_COUNT],
    /// All finite |gyro| samples, ascending. Source of the ECDF plot.
    pub sorted_abs: Vec<f64>,
}

/// Per-axis |gyro| distributions. A series is `None` when the axis has no finite sample.
pub struct VibrationPercentileResult {
    pub axis_name: String,
    pub filtered: Option<VibrationSeries>,
    pub unfiltered: Option<VibrationSeries>,
}

/// Value at percentile `pct` (0..=100) of ascending `sorted`: `sorted[(n - 1) * pct / 100]`
/// with integer division. `None` when `sorted` is empty. The motor-desync and stick-distribution
/// percentiles are private to their modules and use a float index; this one needs an exact index
/// for whole-number percentiles.
pub fn percentile_of_sorted(sorted: &[f64], pct: usize) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    // u128 product cannot overflow, also on 32-bit usize; the quotient is <= len - 1.
    let idx = ((sorted.len() - 1) as u128 * pct.min(100) as u128 / 100) as usize;
    Some(sorted[idx])
}

/// Builds a series from raw samples: drops non-finite values, takes |x|, sorts ascending.
/// `None` when no finite sample remains.
pub fn build_series(samples: impl Iterator<Item = f64>) -> Option<VibrationSeries> {
    let mut sorted_abs: Vec<f64> = samples.filter(|v| v.is_finite()).map(f64::abs).collect();
    if sorted_abs.is_empty() {
        return None;
    }
    sorted_abs.sort_by(|a, b| a.total_cmp(b));
    let mut percentiles = [0.0; PERCENTILE_COUNT];
    for (slot, pct) in percentiles.iter_mut().zip(VIBRATION_PERCENTILES_PCT) {
        *slot = percentile_of_sorted(&sorted_abs, pct)?;
    }
    Some(VibrationSeries {
        count: sorted_abs.len(),
        percentiles,
        sorted_abs,
    })
}

/// Per-axis |gyro| and |gyroUnfilt| distributions for the console, report and plot.
pub fn analyze_vibration_percentiles(log_data: &[LogRowData]) -> Vec<VibrationPercentileResult> {
    (0..AXIS_NAMES.len())
        .map(|axis| VibrationPercentileResult {
            axis_name: AXIS_NAMES[axis].to_string(),
            filtered: build_series(log_data.iter().filter_map(|r| r.gyro[axis])),
            unfiltered: build_series(log_data.iter().filter_map(|r| r.gyro_unfilt[axis])),
        })
        .collect()
}

/// One percentile value as shown in every output (deg/s, one decimal).
pub fn format_percentile_value(value: f64) -> String {
    format!("{value:.1}")
}

/// `P50 a / P90 b / P95 c / P99 d deg/s`: the single text for console and plot legends.
pub fn percentile_summary_text(series: &VibrationSeries) -> String {
    let parts: Vec<String> = VIBRATION_PERCENTILES_PCT
        .iter()
        .zip(series.percentiles.iter())
        .map(|(pct, v)| format!("P{pct} {}", format_percentile_value(*v)))
        .collect();
    format!("{} deg/s", parts.join(" / "))
}

/// Console lines for the analysis, one per axis and signal present.
pub fn console_lines(results: &[VibrationPercentileResult]) -> Vec<String> {
    let mut lines = Vec::new();
    for r in results {
        for (signal, series) in [("Filtered", &r.filtered), ("Unfiltered", &r.unfiltered)] {
            if let Some(s) = series {
                lines.push(format!(
                    "  {} {} |gyro| ({} samples): {}",
                    r.axis_name,
                    signal,
                    s.count,
                    percentile_summary_text(s)
                ));
            }
        }
    }
    lines
}

/// ECDF points `(value, cumulative probability)` of ascending `sorted`, at most `max_points`.
/// Point i carries probability `(i + 1) / n`; first and last samples are always kept.
pub fn ecdf_points(sorted: &[f64], max_points: usize) -> Vec<(f64, f64)> {
    let n = sorted.len();
    if n == 0 || max_points == 0 {
        return Vec::new();
    }
    let prob = |i: usize| (i + 1) as f64 / n as f64;
    if n <= max_points || max_points < 2 {
        return (0..n).map(|i| (sorted[i], prob(i))).collect();
    }
    (0..max_points)
        .map(|k| {
            let i = k * (n - 1) / (max_points - 1);
            (sorted[i], prob(i))
        })
        .collect()
}

/// ECDF points of a series, capped at `VIBRATION_ECDF_MAX_POINTS`.
pub fn series_ecdf_points(series: &VibrationSeries) -> Vec<(f64, f64)> {
    ecdf_points(&series.sorted_abs, VIBRATION_ECDF_MAX_POINTS)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_to_hundred() -> Vec<f64> {
        (1..=100).map(f64::from).collect()
    }

    // Definition: value at percentile p is sorted[(n-1)*p/100]; for 1..=100 that is the value p.
    #[test]
    fn known_sample_gives_exact_percentiles() {
        let s = build_series(one_to_hundred().into_iter()).expect("series");
        assert_eq!(s.count, 100);
        assert_eq!(s.percentiles, [50.0, 90.0, 95.0, 99.0]);
    }

    // Definition on 0..=100 (n=101): index (n-1)*p/100 = p, value p.
    #[test]
    fn index_has_no_float_rounding_error() {
        let sorted: Vec<f64> = (0..=100).map(f64::from).collect();
        for pct in 0..=100usize {
            assert_eq!(percentile_of_sorted(&sorted, pct), Some(pct as f64));
        }
    }

    // Definition on n=2: index (n-1)*p/100 is 0 for p<100 and 1 for p=100.
    #[test]
    fn two_samples_pick_lower_until_p100() {
        let s = [1.0, 2.0];
        for pct in [50, 90, 95, 99] {
            assert_eq!(percentile_of_sorted(&s, pct), Some(1.0));
        }
        assert_eq!(percentile_of_sorted(&s, 100), Some(2.0));
        assert_eq!(percentile_of_sorted(&s, 0), Some(1.0));
    }

    #[test]
    fn pct_above_100_clamps_and_huge_pct_does_not_overflow() {
        let s = [1.0, 2.0, 3.0];
        assert_eq!(percentile_of_sorted(&s, 101), Some(3.0));
        assert_eq!(percentile_of_sorted(&s, usize::MAX), Some(3.0));
    }

    #[test]
    fn unsorted_negative_input_is_sorted_by_magnitude() {
        let s = build_series([-3.0, 1.0, -2.0, 4.0].into_iter()).expect("series");
        assert_eq!(s.sorted_abs, [1.0, 2.0, 3.0, 4.0]);
        // n=4: indexes 3*50/100=1, 3*90/100=2, 3*95/100=2, 3*99/100=2
        assert_eq!(s.percentiles, [2.0, 3.0, 3.0, 3.0]);
    }

    #[test]
    fn empty_input_has_no_series() {
        assert!(build_series(std::iter::empty()).is_none());
        assert_eq!(percentile_of_sorted(&[], 90), None);
        assert!(ecdf_points(&[], 10).is_empty());
    }

    #[test]
    fn single_sample_fills_every_percentile() {
        let s = build_series([-7.5].into_iter()).expect("series");
        assert_eq!(s.count, 1);
        assert_eq!(s.percentiles, [7.5; 4]);
        assert_eq!(ecdf_points(&s.sorted_abs, 10), [(7.5, 1.0)]);
    }

    #[test]
    fn non_finite_samples_are_dropped() {
        let s = build_series([f64::NAN, 2.0, f64::INFINITY, f64::NEG_INFINITY, 4.0].into_iter())
            .expect("series");
        assert_eq!(s.count, 2);
        assert_eq!(s.sorted_abs, [2.0, 4.0]);
        assert!(build_series([f64::NAN, f64::NAN].into_iter()).is_none());
    }

    #[test]
    fn ecdf_probability_is_rank_over_n_and_ends_at_one() {
        let pts = ecdf_points(&[1.0, 2.0, 3.0, 4.0], 100);
        assert_eq!(pts, [(1.0, 0.25), (2.0, 0.5), (3.0, 0.75), (4.0, 1.0)]);
    }

    #[test]
    fn ecdf_decimation_keeps_endpoints_and_cap() {
        let sorted = one_to_hundred();
        let pts = ecdf_points(&sorted, 10);
        assert_eq!(pts.len(), 10);
        assert_eq!(pts[0], (1.0, 0.01));
        assert_eq!(pts[9], (100.0, 1.0));
        assert!(pts.windows(2).all(|w| w[0].0 < w[1].0 && w[0].1 < w[1].1));
    }

    #[test]
    fn analyze_keeps_axes_and_signals_separate() {
        let rows: Vec<LogRowData> = (1..=100)
            .map(|i| LogRowData {
                gyro: [Some(f64::from(i)), None, Some(f64::NAN)],
                gyro_unfilt: [Some(f64::from(2 * i)), None, None],
                ..Default::default()
            })
            .collect();
        let res = analyze_vibration_percentiles(&rows);
        assert_eq!(res.len(), AXIS_NAMES.len());
        assert_eq!(res[0].axis_name, "Roll");
        assert_eq!(res[0].filtered.as_ref().expect("f").percentiles[1], 90.0);
        assert_eq!(res[0].unfiltered.as_ref().expect("u").percentiles[1], 180.0);
        assert!(res[1].filtered.is_none() && res[1].unfiltered.is_none());
        assert!(res[2].filtered.is_none());
    }

    #[test]
    fn summary_text_is_shared_by_console_lines() {
        let s = build_series(one_to_hundred().into_iter()).expect("series");
        let text = percentile_summary_text(&s);
        assert_eq!(text, "P50 50.0 / P90 90.0 / P95 95.0 / P99 99.0 deg/s");
        let res = vec![VibrationPercentileResult {
            axis_name: "Roll".to_string(),
            filtered: Some(s),
            unfiltered: None,
        }];
        let lines = console_lines(&res);
        assert_eq!(lines.len(), 1);
        assert!(lines[0].contains(&text), "{}", lines[0]);
    }
}
