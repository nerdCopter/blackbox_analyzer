// src/data_analysis/feedforward.rs

use crate::axis_names::{AXIS_COUNT, AXIS_NAMES};
use crate::constants::{BETAFLIGHT_PTERM_SCALE, FEEDFORWARD_ACTIVE_EPSILON};
use crate::data_input::log_data::LogRowData;
use crate::data_input::pid_metadata::{FirmwareType, PidMetadata};

/// Feedforward contribution of one axis, expressed as the nominal setpoint-error the P term
/// would need to produce the same output.
///
/// Logged `axisP` and `axisF` share one unit (PID controller output). Betaflight computes
/// `axisP = P_gain * PTERM_SCALE * (setpoint - gyro)`, so `axisF / (P_gain * PTERM_SCALE)` is
/// the nominal error in deg/s that would make P push as hard as F does. This is a unit
/// conversion for display, not a model of how the aircraft would fly with feedforward off.
/// In-flight P also varies with TPA, anti-gravity, the yaw P filter and crash recovery; none
/// of these is applied.
#[derive(Debug, Clone, PartialEq)]
pub struct FeedforwardResult {
    pub axis_name: String,
    /// Samples (setpoint, gyro and axisF all present) the figures below cover.
    pub sample_count: usize,
    /// Percent of those samples with non-zero axisF.
    pub active_pct: f64,
    /// RMS of the P-equivalent feedforward, deg/s.
    pub ff_equivalent_rms: f64,
    /// RMS of the tracking error (setpoint - gyro), deg/s.
    pub error_rms: f64,
}

impl FeedforwardResult {
    /// RMS feedforward divided by RMS tracking error; `None` when the error RMS is zero.
    pub fn ff_to_error_ratio(&self) -> Option<f64> {
        (self.error_rms > 0.0).then_some(self.ff_equivalent_rms / self.error_rms)
    }
}

/// deg/s of P-equivalent error per logged axisF unit: `1 / (P_gain * PTERM_SCALE)`.
/// `None` unless the firmware is Betaflight (the scale comes from its source) and the axis
/// has a positive P gain.
pub fn ff_equivalent_scale(pid_metadata: &PidMetadata, axis_index: usize) -> Option<f64> {
    if *pid_metadata.get_firmware_type() != FirmwareType::Betaflight {
        return None;
    }
    let p = pid_metadata.get_axis(axis_index)?.p?;
    (p > 0).then(|| 1.0 / (f64::from(p) * BETAFLIGHT_PTERM_SCALE))
}

/// True when the axis logged at least one non-zero axisF sample.
pub fn axis_has_feedforward(log_data: &[LogRowData], axis_index: usize) -> bool {
    log_data.iter().any(|row| {
        row.f_term[axis_index]
            .is_some_and(|f| f.is_finite() && f.abs() > FEEDFORWARD_ACTIVE_EPSILON)
    })
}

/// Per-axis feedforward figures. An axis is omitted when it has no scale (see
/// `ff_equivalent_scale`), has no complete sample with non-zero axisF, or the sums overflow.
pub fn analyze_feedforward(
    log_data: &[LogRowData],
    pid_metadata: &PidMetadata,
) -> Vec<FeedforwardResult> {
    let mut results = Vec::new();
    for axis_index in 0..AXIS_COUNT.min(AXIS_NAMES.len()) {
        let Some(scale) = ff_equivalent_scale(pid_metadata, axis_index) else {
            continue;
        };
        let (mut n, mut active) = (0usize, 0usize);
        let (mut ff_sq, mut err_sq) = (0.0f64, 0.0f64);
        for row in log_data {
            let (Some(sp), Some(gyro), Some(f)) = (
                row.setpoint[axis_index],
                row.gyro[axis_index],
                row.f_term[axis_index],
            ) else {
                continue;
            };
            if !(sp.is_finite() && gyro.is_finite() && f.is_finite()) {
                continue;
            }
            n += 1;
            if f.abs() > FEEDFORWARD_ACTIVE_EPSILON {
                active += 1;
            }
            ff_sq += (f * scale).powi(2);
            err_sq += (sp - gyro).powi(2);
        }
        // Activity is judged on the rows the figures cover, so an active sample that lacks
        // setpoint or gyro cannot enable a result that describes other rows; overflow of the
        // squared sums (finite inputs near f64::MAX) gives no result.
        if active == 0 || !ff_sq.is_finite() || !err_sq.is_finite() {
            continue;
        }
        let nf = n as f64;
        results.push(FeedforwardResult {
            axis_name: AXIS_NAMES[axis_index].to_string(),
            sample_count: n,
            active_pct: 100.0 * active as f64 / nf,
            ff_equivalent_rms: (ff_sq / nf).sqrt(),
            error_rms: (err_sq / nf).sqrt(),
        });
    }
    results
}

/// One-line summary shared by the console and the report.
pub fn feedforward_summary_text(r: &FeedforwardResult) -> String {
    let ratio = r
        .ff_to_error_ratio()
        .map_or("N/A".to_string(), |v| format!("{v:.2}"));
    format!(
        "FF active {:.1}% / FF-equivalent RMS {:.1} deg/s / error RMS {:.1} deg/s / FF:error {}",
        r.active_pct, r.ff_equivalent_rms, r.error_rms, ratio
    )
}

/// Console lines, one per axis.
pub fn console_lines(results: &[FeedforwardResult]) -> Vec<String> {
    results
        .iter()
        .map(|r| format!("{}: {}", r.axis_name, feedforward_summary_text(r)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_input::pid_metadata::parse_pid_metadata;

    fn meta(firmware: &str, roll_pid: &str) -> PidMetadata {
        parse_pid_metadata(&[
            ("Firmware revision".to_string(), firmware.to_string()),
            ("rollPID".to_string(), roll_pid.to_string()),
        ])
    }

    fn row(sp: f64, gyro: f64, f: f64) -> LogRowData {
        let mut r = LogRowData::default();
        r.setpoint[0] = Some(sp);
        r.gyro[0] = Some(gyro);
        r.f_term[0] = Some(f);
        r
    }

    // Spec: scale = 1 / (P * PTERM_SCALE); P 50 -> 1 / (50 * 0.032029).
    #[test]
    fn scale_follows_p_gain_and_firmware() {
        let bf = meta("Betaflight 4.5.2 (024f8e13d) STM32F405", "50,80,40");
        let scale = ff_equivalent_scale(&bf, 0).expect("scale");
        assert!((scale - 1.0 / (50.0 * 0.032029)).abs() < 1e-12);
        // Yaw has no P in this header set, so there is no scale.
        assert!(bf.get_axis(2).unwrap().p.is_none());
        assert!(ff_equivalent_scale(&bf, 2).is_none());
        let ef = meta("EmuFlight 0.4.3 (abc) STM32F405", "50,80,40");
        assert!(ff_equivalent_scale(&ef, 0).is_none());
        let zero_p = meta("Betaflight 4.5.2 (024f8e13d) STM32F405", "0,80,40");
        assert!(ff_equivalent_scale(&zero_p, 0).is_none());
    }

    // Spec: constant F = 3.2029 with P 100 is 3.2029 / (100 * 0.032029) = 1 deg/s everywhere;
    // constant error 4 gives error RMS 4 and ratio 0.25; F is active on every sample.
    #[test]
    fn constant_signals_give_exact_rms() {
        let md = meta("Betaflight 4.5.2 (024f8e13d) STM32F405", "100,80,40");
        let rows: Vec<_> = (0..10).map(|_| row(14.0, 10.0, 3.2029)).collect();
        let out = analyze_feedforward(&rows, &md);
        assert_eq!(out.len(), 1);
        let r = &out[0];
        assert_eq!(r.axis_name, "Roll");
        assert_eq!(r.sample_count, 10);
        assert!((r.active_pct - 100.0).abs() < 1e-9);
        assert!((r.ff_equivalent_rms - 1.0).abs() < 1e-9);
        assert!((r.error_rms - 4.0).abs() < 1e-9);
        assert!((r.ff_to_error_ratio().unwrap() - 0.25).abs() < 1e-9);
    }

    // Spec: half the samples have F, so active is 50%; zero error leaves the ratio undefined.
    #[test]
    fn active_fraction_and_zero_error() {
        let md = meta("Betaflight 4.5.2 (024f8e13d) STM32F405", "100,80,40");
        let rows = vec![row(5.0, 5.0, 0.0), row(5.0, 5.0, 3.2029)];
        let r = &analyze_feedforward(&rows, &md)[0];
        assert!((r.active_pct - 50.0).abs() < 1e-9);
        assert_eq!(r.error_rms, 0.0);
        assert!(r.ff_to_error_ratio().is_none());
        assert!(feedforward_summary_text(r).ends_with("FF:error N/A"));
    }

    // Spec: no non-zero axisF (feedforward off, or the field logged as all zero) gives no result;
    // so do incomplete rows and non-finite samples.
    #[test]
    fn no_feedforward_or_unusable_samples_give_no_result() {
        let md = meta("Betaflight 4.5.2 (024f8e13d) STM32F405", "100,80,40");
        let zeros: Vec<_> = (0..5).map(|_| row(5.0, 4.0, 0.0)).collect();
        assert!(analyze_feedforward(&zeros, &md).is_empty());
        let mut missing = row(5.0, 4.0, 1.0);
        missing.gyro[0] = None;
        let nan = row(f64::NAN, 4.0, 1.0);
        assert!(analyze_feedforward(&[missing, nan], &md).is_empty());
        // The only active sample lacks gyro; the complete row has F = 0.
        let mut active_incomplete = row(5.0, 4.0, 1.0);
        active_incomplete.gyro[0] = None;
        assert!(analyze_feedforward(&[active_incomplete, row(5.0, 4.0, 0.0)], &md).is_empty());
        // Finite inputs whose squares overflow give no result.
        assert!(analyze_feedforward(&[row(1e200, 0.0, 1e200)], &md).is_empty());
        assert!(analyze_feedforward(&[], &md).is_empty());
    }
}
