// src/data_analysis/motor_symmetry.rs
//
// Per-motor command distribution and imbalance. Reuses no existing primitive: the percentile
// helpers in stick_distribution.rs (private, P95 only) and filter_response.rs (rounded index on a
// pooled series) do not give per-motor quartiles with the sorted-index rule used here.

use crate::constants::{
    MOTOR_SYMMETRY_IDX_MEDIAN, MOTOR_SYMMETRY_IDX_Q1, MOTOR_SYMMETRY_IDX_Q3,
    MOTOR_SYMMETRY_IDX_WHISKER_HIGH, MOTOR_SYMMETRY_IDX_WHISKER_LOW,
    MOTOR_SYMMETRY_PERCENTILES_PCT, RATIO_TO_PERCENT,
};
use crate::data_input::log_data::LogRowData;

/// Distribution of one motor's command over the analyzed samples, in the log's raw command units.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorCommandStats {
    /// Position in `LogRowData::motors` (the motor[N] columns in channel order).
    pub motor_idx: usize,
    /// Number of finite samples.
    pub count: usize,
    pub mean: f64,
    /// Values at `MOTOR_SYMMETRY_PERCENTILES_PCT`, in the same order.
    pub percentiles: [f64; MOTOR_SYMMETRY_PERCENTILES_PCT.len()],
    /// `(mean - fleet_mean) / fleet_mean * 100`; `None` when the fleet mean is not positive.
    pub deviation_pct: Option<f64>,
}

impl MotorCommandStats {
    pub fn whisker_low(&self) -> f64 {
        self.percentiles[MOTOR_SYMMETRY_IDX_WHISKER_LOW]
    }
    pub fn q1(&self) -> f64 {
        self.percentiles[MOTOR_SYMMETRY_IDX_Q1]
    }
    pub fn median(&self) -> f64 {
        self.percentiles[MOTOR_SYMMETRY_IDX_MEDIAN]
    }
    pub fn q3(&self) -> f64 {
        self.percentiles[MOTOR_SYMMETRY_IDX_Q3]
    }
    pub fn whisker_high(&self) -> f64 {
        self.percentiles[MOTOR_SYMMETRY_IDX_WHISKER_HIGH]
    }
}

/// Imbalance across motors: `(max mean - min mean) / fleet mean * 100`, where the fleet mean is
/// the mean of the per-motor means. Needs at least two motors with samples and a positive fleet
/// mean.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorImbalance {
    pub imbalance_pct: f64,
    /// `max mean - min mean`, raw command units.
    pub mean_spread: f64,
    pub fleet_mean: f64,
    pub highest_motor_idx: usize,
    pub lowest_motor_idx: usize,
}

/// Result of the motor symmetry analysis. `motors` is empty when the log has no motor data.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MotorSymmetryResult {
    pub motors: Vec<MotorCommandStats>,
    pub imbalance: Option<MotorImbalance>,
}

/// Value at percentile `pct` of the ascending-sorted `sorted`: `sorted[(n - 1) * pct / 100]`
/// (integer division). `None` for an empty slice.
fn percentile_sorted(sorted: &[f64], pct: usize) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    sorted
        .get((sorted.len() - 1) * pct / RATIO_TO_PERCENT as usize)
        .copied()
}

/// Statistics of one motor's finite samples, without the fleet-relative deviation. `None` when
/// there are no finite samples.
fn motor_stats(motor_idx: usize, mut samples: Vec<f64>) -> Option<MotorCommandStats> {
    samples.retain(|v| v.is_finite());
    if samples.is_empty() {
        return None;
    }
    samples.sort_by(|a, b| a.total_cmp(b));
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    let mut percentiles = [0.0; MOTOR_SYMMETRY_PERCENTILES_PCT.len()];
    for (slot, pct) in percentiles
        .iter_mut()
        .zip(MOTOR_SYMMETRY_PERCENTILES_PCT.iter())
    {
        *slot = percentile_sorted(&samples, *pct)?;
    }
    Some(MotorCommandStats {
        motor_idx,
        count: samples.len(),
        mean,
        percentiles,
        deviation_pct: None,
    })
}

/// Builds the result from per-motor sample sets (index = position in `LogRowData::motors`).
fn analyze_samples(per_motor: Vec<Vec<f64>>) -> MotorSymmetryResult {
    let mut motors: Vec<MotorCommandStats> = per_motor
        .into_iter()
        .enumerate()
        .filter_map(|(idx, samples)| motor_stats(idx, samples))
        .collect();

    let imbalance = if motors.len() >= 2 {
        let fleet_mean = motors.iter().map(|m| m.mean).sum::<f64>() / motors.len() as f64;
        if fleet_mean > 0.0 {
            for m in &mut motors {
                m.deviation_pct = Some((m.mean - fleet_mean) / fleet_mean * RATIO_TO_PERCENT);
            }
            let highest = motors
                .iter()
                .max_by(|a, b| a.mean.total_cmp(&b.mean))
                .map(|m| (m.motor_idx, m.mean));
            let lowest = motors
                .iter()
                .min_by(|a, b| a.mean.total_cmp(&b.mean))
                .map(|m| (m.motor_idx, m.mean));
            match (highest, lowest) {
                (Some((hi_idx, hi)), Some((lo_idx, lo))) => Some(MotorImbalance {
                    imbalance_pct: (hi - lo) / fleet_mean * RATIO_TO_PERCENT,
                    mean_spread: hi - lo,
                    fleet_mean,
                    highest_motor_idx: hi_idx,
                    lowest_motor_idx: lo_idx,
                }),
                _ => None,
            }
        } else {
            None
        }
    } else {
        None
    };

    MotorSymmetryResult { motors, imbalance }
}

/// Per-motor command distribution and imbalance over all rows of `log_data`. Each motor uses its
/// own finite samples. Motors beyond four (6- and 8-motor craft) are included as logged.
pub fn analyze_motor_symmetry(log_data: &[LogRowData]) -> MotorSymmetryResult {
    let motor_count = log_data.iter().map(|r| r.motors.len()).max().unwrap_or(0);
    let mut per_motor: Vec<Vec<f64>> = vec![Vec::new(); motor_count];
    for row in log_data {
        for (idx, v) in row.motors.iter().enumerate() {
            if let Some(v) = v {
                per_motor[idx].push(*v);
            }
        }
    }
    analyze_samples(per_motor)
}

/// One-line imbalance summary shared by the console, the report and the plot title.
pub fn imbalance_summary_text(result: &MotorSymmetryResult) -> String {
    match &result.imbalance {
        Some(i) => format!(
            "Imbalance {:.1}% of fleet mean {:.0} (Motor {} highest, Motor {} lowest, mean spread {:.0})",
            i.imbalance_pct, i.fleet_mean, i.highest_motor_idx, i.lowest_motor_idx, i.mean_spread
        ),
        None if result.motors.len() < 2 => {
            "Imbalance not computed (fewer than two motors with samples)".to_string()
        }
        None => "Imbalance not computed (fleet mean command is not positive)".to_string(),
    }
}

/// Deviation of one motor's mean from the fleet mean, shared by the console and the report.
pub fn deviation_text(m: &MotorCommandStats) -> String {
    match m.deviation_pct {
        Some(d) => format!("{d:+.1}% vs fleet mean"),
        None => "no fleet comparison".to_string(),
    }
}

/// One line per motor, shared by the console and the report table cells.
pub fn motor_summary_text(m: &MotorCommandStats) -> String {
    format!(
        "Motor {}: mean {:.0} ({}), median {:.0}, P25-P75 {:.0}-{:.0}, P5-P95 {:.0}-{:.0}, {} samples",
        m.motor_idx,
        m.mean,
        deviation_text(m),
        m.median(),
        m.q1(),
        m.q3(),
        m.whisker_low(),
        m.whisker_high(),
        m.count
    )
}

/// Console lines: one per motor, then the imbalance summary. Empty when no motor has samples.
pub fn console_lines(result: &MotorSymmetryResult) -> Vec<String> {
    if result.motors.is_empty() {
        return Vec::new();
    }
    let mut lines: Vec<String> = result.motors.iter().map(motor_summary_text).collect();
    lines.push(imbalance_summary_text(result));
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(motors: &[Option<f64>]) -> LogRowData {
        LogRowData {
            motors: motors.to_vec(),
            ..Default::default()
        }
    }

    fn constant_rows(values: &[f64], n: usize) -> Vec<LogRowData> {
        (0..n)
            .map(|_| row(&values.iter().map(|v| Some(*v)).collect::<Vec<_>>()))
            .collect()
    }

    // Spec: equal motors give zero imbalance and zero deviation.
    #[test]
    fn equal_motors_have_zero_imbalance() {
        let r = analyze_motor_symmetry(&constant_rows(&[500.0; 4], 10));
        assert_eq!(r.motors.len(), 4);
        let i = r.imbalance.expect("imbalance");
        assert_eq!(i.imbalance_pct, 0.0);
        assert_eq!(i.mean_spread, 0.0);
        for m in &r.motors {
            assert_eq!(m.deviation_pct, Some(0.0));
            assert_eq!(m.median(), 500.0);
        }
    }

    // Spec: motors 100,100,100,120 -> fleet mean 105, spread 20, imbalance 20/105*100.
    #[test]
    fn one_offset_motor_gives_the_known_imbalance() {
        let r = analyze_motor_symmetry(&constant_rows(&[100.0, 100.0, 100.0, 120.0], 5));
        let i = r.imbalance.expect("imbalance");
        assert_eq!(i.fleet_mean, 105.0);
        assert_eq!(i.mean_spread, 20.0);
        assert!((i.imbalance_pct - 20.0 / 105.0 * 100.0).abs() < 1e-9);
        assert_eq!(i.highest_motor_idx, 3);
        assert_eq!(i.lowest_motor_idx, 0);
        let dev3 = r.motors[3].deviation_pct.expect("deviation");
        assert!((dev3 - 15.0 / 105.0 * 100.0).abs() < 1e-9);
    }

    // Spec: 1..=100 gives P5 5, P25 25, P50 50, P75 75, P95 95 (index (n-1)*p/100) and mean 50.5.
    #[test]
    fn percentiles_follow_the_sorted_index_rule() {
        let samples: Vec<f64> = (1..=100).map(f64::from).collect();
        let s = motor_stats(0, samples).expect("stats");
        assert_eq!(s.percentiles, [5.0, 25.0, 50.0, 75.0, 95.0]);
        assert_eq!(s.mean, 50.5);
        assert_eq!(s.count, 100);
    }

    // Spec: the order of the input samples does not change the distribution.
    #[test]
    fn unsorted_input_matches_sorted_input() {
        let sorted: Vec<f64> = (1..=100).map(f64::from).collect();
        let reversed: Vec<f64> = sorted.iter().rev().copied().collect();
        assert_eq!(motor_stats(0, sorted), motor_stats(0, reversed));
    }

    // Spec: no rows, rows with no motors, and a motor with only missing samples give no result.
    #[test]
    fn empty_and_missing_inputs_give_no_motors() {
        assert!(analyze_motor_symmetry(&[]).motors.is_empty());
        assert!(analyze_motor_symmetry(&[row(&[])]).motors.is_empty());
        let r = analyze_motor_symmetry(&[row(&[None, None])]);
        assert!(r.motors.is_empty());
        assert!(r.imbalance.is_none());
        assert!(console_lines(&r).is_empty());
    }

    // Spec: one sample gives that value for every percentile and no imbalance (one motor).
    #[test]
    fn single_sample_single_motor() {
        let r = analyze_motor_symmetry(&[row(&[Some(700.0)])]);
        assert_eq!(r.motors.len(), 1);
        assert_eq!(r.motors[0].percentiles, [700.0; 5]);
        assert_eq!(r.motors[0].mean, 700.0);
        assert!(r.imbalance.is_none());
        assert!(imbalance_summary_text(&r).contains("fewer than two"));
    }

    // Spec: one sample per motor already gives an imbalance; two samples give P25/P75 at index 0.
    #[test]
    fn one_sample_per_motor_and_two_sample_quartiles() {
        let r = analyze_motor_symmetry(&[row(&[Some(100.0), Some(200.0)])]);
        let i = r.imbalance.expect("imbalance");
        assert!((i.imbalance_pct - 100.0 / 150.0 * 100.0).abs() < 1e-9);
        let two = motor_stats(0, vec![200.0, 100.0]).expect("stats");
        assert_eq!(two.q1(), 100.0);
        assert_eq!(two.q3(), 100.0);
        assert_eq!(two.whisker_high(), 100.0);
    }

    // Spec: non-finite samples are dropped; a missing sample in one row does not shift others.
    #[test]
    fn non_finite_and_missing_samples_are_skipped() {
        let rows = vec![
            row(&[Some(100.0), Some(f64::NAN)]),
            row(&[None, Some(300.0)]),
            row(&[Some(300.0), Some(f64::INFINITY)]),
        ];
        let r = analyze_motor_symmetry(&rows);
        assert_eq!(r.motors[0].count, 2);
        assert_eq!(r.motors[0].mean, 200.0);
        assert_eq!(r.motors[1].count, 1);
        assert_eq!(r.motors[1].mean, 300.0);
    }

    // Spec: a motor with no samples is left out and keeps the others' original indices.
    #[test]
    fn missing_motor_keeps_other_indices() {
        let rows = vec![row(&[Some(100.0), None, Some(200.0)])];
        let r = analyze_motor_symmetry(&rows);
        let idx: Vec<usize> = r.motors.iter().map(|m| m.motor_idx).collect();
        assert_eq!(idx, vec![0, 2]);
        let i = r.imbalance.expect("imbalance");
        assert_eq!((i.highest_motor_idx, i.lowest_motor_idx), (2, 0));
    }

    // Spec: 6- and 8-motor logs are covered, rows may have more motors than the first row.
    #[test]
    fn six_and_eight_motor_logs() {
        let six = analyze_motor_symmetry(&constant_rows(&[10.0, 10.0, 10.0, 10.0, 10.0, 40.0], 3));
        assert_eq!(six.motors.len(), 6);
        assert_eq!(six.imbalance.expect("imbalance").highest_motor_idx, 5);
        let eight = analyze_motor_symmetry(&constant_rows(&[10.0; 8], 3));
        assert_eq!(eight.motors.len(), 8);
        assert_eq!(eight.imbalance.expect("imbalance").imbalance_pct, 0.0);
    }

    // Spec: a non-positive fleet mean gives no imbalance and no deviation.
    #[test]
    fn zero_fleet_mean_gives_no_imbalance() {
        let r = analyze_motor_symmetry(&constant_rows(&[0.0, 0.0], 4));
        assert!(r.imbalance.is_none());
        assert!(r.motors.iter().all(|m| m.deviation_pct.is_none()));
        assert!(imbalance_summary_text(&r).contains("not positive"));
    }

    // Spec: console text carries the shared summary strings.
    #[test]
    fn console_lines_use_the_shared_text() {
        let r = analyze_motor_symmetry(&constant_rows(&[100.0, 100.0, 100.0, 120.0], 5));
        let lines = console_lines(&r);
        assert_eq!(lines.len(), 5);
        assert_eq!(lines[0], motor_summary_text(&r.motors[0]));
        assert_eq!(lines[4], imbalance_summary_text(&r));
        assert!(lines[4].starts_with("Imbalance 19.0% of fleet mean 105"));
    }
}
