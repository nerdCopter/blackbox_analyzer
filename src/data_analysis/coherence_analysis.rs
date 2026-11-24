// src/data_analysis/coherence_analysis.rs

use crate::axis_names::AXIS_NAMES;
use crate::constants::{
    COHERENCE_MEDIUM_THRESHOLD, COHERENCE_MIN_SEGMENTS, COHERENCE_MIN_SIGNAL_STD,
    COHERENCE_OVERLAP_FRACTION, COHERENCE_SEGMENT_SECONDS, COHERENCE_SUMMARY_MAX_HZ, PSD_EPSILON,
};
use crate::data_analysis::spectral_analysis::{coherence, welch_cpsd, welch_psd, WelchConfig};
use crate::data_analysis::transfer_function_estimation::calculate_signal_variation;
use crate::data_input::log_data::LogRowData;

/// Magnitude-squared coherence between setpoint (input) and gyro (output) for one axis.
/// Built on the shared Welch PSD/CPSD estimates and `spectral_analysis::coherence`.
#[derive(Debug, Clone)]
pub struct CoherenceResult {
    pub axis_name: String,
    /// Frequencies in Hz of the defined bins, ascending (DC and bins without signal power omitted).
    pub frequency_hz: Vec<f64>,
    /// Coherence per bin, 0 to 1.
    pub coherence: Vec<f64>,
    /// Averaged Welch segments behind every bin.
    pub segment_count: usize,
    /// Mean coherence over the defined bins with `f <= COHERENCE_SUMMARY_MAX_HZ`, and the
    /// lowest and highest frequency of those bins; `None` when no bin falls there.
    pub mean_coherence: Option<(f64, f64, f64)>,
    /// Highest frequency reached by the unbroken run of bins at or above
    /// `COHERENCE_MEDIUM_THRESHOLD`, starting at the first bin above DC. `None` when that
    /// first bin is already below the threshold.
    pub coherent_bandwidth_hz: Option<f64>,
}

/// Welch segment length in samples for a sample rate: next power of two of
/// `sample_rate * COHERENCE_SEGMENT_SECONDS`.
pub fn segment_length_for(sample_rate: f64) -> usize {
    ((sample_rate * COHERENCE_SEGMENT_SECONDS).round().max(1.0) as usize).next_power_of_two()
}

/// Welch hop size in samples, as `welch_psd` computes it.
fn hop_size(segment_length: usize) -> usize {
    ((1.0 - COHERENCE_OVERLAP_FRACTION) * segment_length as f64) as usize
}

/// Number of Welch segments that fit `len` samples; 0 when `len` is shorter than one segment.
pub fn segment_count_for(len: usize, segment_length: usize) -> usize {
    let hop = hop_size(segment_length);
    if hop == 0 || len < segment_length {
        return 0;
    }
    (len - segment_length) / hop + 1
}

/// Magnitude-squared coherence of `input` and `output` (equal length) per frequency bin.
/// Errors on a non-positive sample rate, unequal or too-short input (fewer than
/// `COHERENCE_MIN_SEGMENTS` segments), or a constant signal.
pub fn estimate_coherence(
    input: &[f32],
    output: &[f32],
    sample_rate: f64,
) -> Result<(Vec<f64>, Vec<f64>, usize), String> {
    if !sample_rate.is_finite() || sample_rate <= 0.0 {
        return Err("invalid sample rate".to_string());
    }
    if input.len() != output.len() {
        return Err("signals differ in length".to_string());
    }
    let segment_length = segment_length_for(sample_rate);
    let segments = segment_count_for(input.len(), segment_length);
    if segments < COHERENCE_MIN_SEGMENTS {
        return Err(format!(
            "{} samples give {} Welch segments of {} samples (minimum {})",
            input.len(),
            segments,
            segment_length,
            COHERENCE_MIN_SEGMENTS
        ));
    }
    if calculate_signal_variation(input) < COHERENCE_MIN_SIGNAL_STD {
        return Err("setpoint is constant".to_string());
    }
    if calculate_signal_variation(output) < COHERENCE_MIN_SIGNAL_STD {
        return Err("gyro is constant".to_string());
    }

    let input = remove_mean(input);
    let output = remove_mean(output);
    let config = WelchConfig {
        segment_length,
        overlap_percent: COHERENCE_OVERLAP_FRACTION,
    };
    let cpsd = welch_cpsd(&output, &input, sample_rate, Some(config.clone()))
        .map_err(|e| e.to_string())?;
    let psd_in = welch_psd(&input, sample_rate, Some(config.clone())).map_err(|e| e.to_string())?;
    let psd_out = welch_psd(&output, sample_rate, Some(config)).map_err(|e| e.to_string())?;
    let coh = coherence(&cpsd, &psd_in, &psd_out).map_err(|e| e.to_string())?;
    // Dropped bins: DC (the mean is removed, so it carries no power) and every bin where
    // Sxx * Syy is at or below the guard of `spectral_analysis::coherence` (PSD_EPSILON). There
    // the shared function returns 0 for lack of signal power, which would read as "unrelated"
    // although the ratio is undefined.
    let (mut freqs, mut values) = (Vec::new(), Vec::new());
    for i in 1..coh.len() {
        if psd_in[i].1 * psd_out[i].1 > PSD_EPSILON {
            freqs.push(coh[i].0);
            values.push(coh[i].1);
        }
    }
    if freqs.is_empty() {
        return Err("no frequency bin has signal power in both signals".to_string());
    }
    Ok((freqs, values, segments))
}

fn remove_mean(signal: &[f32]) -> Vec<f32> {
    let mean = signal.iter().map(|&x| x as f64).sum::<f64>() / signal.len() as f64;
    signal.iter().map(|&x| (x as f64 - mean) as f32).collect()
}

/// `(mean, lowest f, highest f)` over the bins with `f <= COHERENCE_SUMMARY_MAX_HZ`.
fn mean_in_band(freq: &[f64], coh: &[f64]) -> Option<(f64, f64, f64)> {
    let band: Vec<(f64, f64)> = freq
        .iter()
        .copied()
        .zip(coh.iter().copied())
        .filter(|&(f, _)| f <= COHERENCE_SUMMARY_MAX_HZ)
        .collect();
    let (first, last) = (band.first()?.0, band.last()?.0);
    Some((
        band.iter().map(|&(_, c)| c).sum::<f64>() / band.len() as f64,
        first,
        last,
    ))
}

/// Highest frequency of the unbroken run of bins at or above `COHERENCE_MEDIUM_THRESHOLD`,
/// starting at the first bin above DC.
fn coherent_bandwidth(freq: &[f64], coh: &[f64]) -> Option<f64> {
    let mut top = None;
    for (&f, &c) in freq.iter().zip(coh).filter(|(&f, _)| f > 0.0) {
        if c >= COHERENCE_MEDIUM_THRESHOLD {
            top = Some(f);
        } else {
            break;
        }
    }
    top
}

/// Coherence for one axis from the paired setpoint and gyro samples of `log_data`.
pub fn analyze_axis(
    log_data: &[LogRowData],
    sample_rate: f64,
    axis: usize,
) -> Result<CoherenceResult, String> {
    let name = AXIS_NAMES
        .get(axis)
        .ok_or_else(|| format!("invalid axis index {axis}"))?;
    let (mut setpoint, mut gyro) = (Vec::new(), Vec::new());
    for row in log_data {
        if let (Some(sp), Some(gy)) = (row.setpoint[axis], row.gyro[axis]) {
            if sp.is_finite() && gy.is_finite() {
                setpoint.push(sp as f32);
                gyro.push(gy as f32);
            }
        }
    }
    let (frequency_hz, coh, segment_count) = estimate_coherence(&setpoint, &gyro, sample_rate)?;
    Ok(CoherenceResult {
        axis_name: name.to_string(),
        mean_coherence: mean_in_band(&frequency_hz, &coh),
        coherent_bandwidth_hz: coherent_bandwidth(&frequency_hz, &coh),
        frequency_hz,
        coherence: coh,
        segment_count,
    })
}

/// Coherence for every axis; an axis that cannot be estimated is skipped with a console note.
pub fn analyze_coherence(
    log_data: &[LogRowData],
    sample_rate: Option<f64>,
) -> Vec<CoherenceResult> {
    let Some(sr) = sample_rate else {
        println!("\nINFO: Skipping Coherence Plot: Sample rate could not be determined.");
        return Vec::new();
    };
    let mut out = Vec::new();
    for (axis, axis_name) in AXIS_NAMES.iter().enumerate() {
        match analyze_axis(log_data, sr, axis) {
            Ok(r) => out.push(r),
            Err(reason) => println!("\nINFO: Skipping Coherence for {axis_name}: {reason}."),
        }
    }
    out
}

/// One-line summary shared by console, report and plot legend.
pub fn summary_text(r: &CoherenceResult) -> String {
    let mean = r
        .mean_coherence
        .map_or("mean N/A".to_string(), |(m, lo, hi)| {
            format!("mean {m:.2} over {lo:.1}-{hi:.1} Hz")
        });
    let bandwidth = r
        .coherent_bandwidth_hz
        .map_or("none".to_string(), |b| format!("{b:.1} Hz"));
    format!("{mean}, coherence >= {COHERENCE_MEDIUM_THRESHOLD:.1} up to {bandwidth}")
}

/// Console lines, one per estimated axis.
pub fn console_lines(results: &[CoherenceResult]) -> Vec<String> {
    results
        .iter()
        .map(|r| {
            format!(
                "  {} setpoint-gyro coherence ({} segments): {}",
                r.axis_name,
                r.segment_count,
                summary_text(r)
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TEST_SAMPLE_RATE: f64 = 1000.0;
    // 512-sample segments (0.5 s at 1 kHz), hop 256: 80 segments need (80 - 1) * 256 + 512 samples.
    const TEST_LEN: usize = 79 * 256 + 512;

    // Amplitude in deg/s. `spectral_analysis::coherence` returns 0 where Sxx * Syy is below an
    // absolute 1e-12, so test signals use gyro-like magnitudes instead of unit-variance values.
    const TEST_AMPLITUDE_DEG_S: f64 = 100.0;

    /// Deterministic white noise, uniform in +-`TEST_AMPLITUDE_DEG_S`, zero mean (LCG, fixed seed).
    fn noise(seed: u64, len: usize) -> Vec<f32> {
        let mut state = seed;
        (0..len)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((state >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) as f32
                    * TEST_AMPLITUDE_DEG_S as f32
            })
            .collect()
    }

    fn mean(v: &[f64]) -> f64 {
        v.iter().sum::<f64>() / v.len() as f64
    }

    // Definition: gamma^2 = |Sxy|^2 / (Sxx Syy); identical signals give |Sxx|^2 / Sxx^2 = 1.
    #[test]
    fn identical_signals_have_unit_coherence() {
        let x = noise(1, TEST_LEN);
        let (f, c, segments) = estimate_coherence(&x, &x, TEST_SAMPLE_RATE).expect("ok");
        assert_eq!(segments, 79 + 1);
        assert_eq!(f.len(), 256);
        assert!((f[0] - TEST_SAMPLE_RATE / 512.0).abs() < 1e-9);
        for (&freq, &v) in f.iter().zip(&c) {
            assert!((v - 1.0).abs() < 1e-4, "f={freq} coherence={v}");
        }
    }

    // Independent noise: coherence estimate bias is about 1 / segments (80 here), so the mean
    // over all bins stays far below 0.1.
    #[test]
    fn independent_noise_has_low_coherence() {
        let x = noise(2, TEST_LEN);
        let y = noise(3, TEST_LEN);
        let (_, c, segments) = estimate_coherence(&x, &y, TEST_SAMPLE_RATE).expect("ok");
        let m = mean(&c);
        assert!(m < 5.0 / segments as f64, "mean coherence {m}");
    }

    // y = x + n with equal-variance white x and n: |H| = 1, Sxx = Snn, so
    // gamma^2 = |H|^2 Sxx / (|H|^2 Sxx + Snn) = 0.5 at every frequency.
    #[test]
    fn unit_gain_plus_equal_noise_gives_one_half() {
        let x = noise(4, TEST_LEN);
        let n = noise(5, TEST_LEN);
        let y: Vec<f32> = x.iter().zip(&n).map(|(a, b)| a + b).collect();
        let (_, c, _) = estimate_coherence(&x, &y, TEST_SAMPLE_RATE).expect("ok");
        let m = mean(&c[..c.len() - 1]);
        assert!((m - 0.5).abs() < 0.05, "mean coherence {m}");
    }

    // y = 3 x + n with Snn = Sxx: gamma^2 = 9 / (9 + 1) = 0.9. A gain changes the transfer
    // function but, for fixed noise, the analytic coherence follows the signal-to-noise ratio.
    #[test]
    fn gain_three_plus_noise_gives_nine_tenths() {
        let x = noise(6, TEST_LEN);
        let n = noise(7, TEST_LEN);
        let y: Vec<f32> = x.iter().zip(&n).map(|(a, b)| 3.0 * a + b).collect();
        let (_, c, _) = estimate_coherence(&x, &y, TEST_SAMPLE_RATE).expect("ok");
        let m = mean(&c[..c.len() - 1]);
        assert!((m - 0.9).abs() < 0.03, "mean coherence {m}");
    }

    // A noise-free one-sample delay is a pure linear system. The window shifts by one sample
    // relative to the signal, which costs coherence only where the phase changes quickly, so
    // check the bins up to 100 Hz (well below the 500 Hz Nyquist limit).
    #[test]
    fn noise_free_delay_keeps_unit_coherence_at_low_frequency() {
        let x = noise(8, TEST_LEN);
        let mut y = vec![0.0f32; TEST_LEN];
        y[1..].copy_from_slice(&x[..TEST_LEN - 1]);
        let (f, c, _) = estimate_coherence(&x, &y, TEST_SAMPLE_RATE).expect("ok");
        let low: Vec<f64> = f
            .iter()
            .zip(&c)
            .filter(|(&f, _)| f <= 100.0)
            .map(|(_, &c)| c)
            .collect();
        assert!(mean(&low) > 0.99, "mean coherence {}", mean(&low));
    }

    #[test]
    fn coherence_stays_within_unit_interval() {
        let x = noise(9, TEST_LEN);
        let y = noise(10, TEST_LEN);
        let (_, c, _) = estimate_coherence(&x, &y, TEST_SAMPLE_RATE).expect("ok");
        assert!(c.iter().all(|v| (0.0..=1.0).contains(v)));
    }

    #[test]
    fn empty_input_is_rejected() {
        assert!(estimate_coherence(&[], &[], TEST_SAMPLE_RATE).is_err());
    }

    #[test]
    fn constant_signal_is_rejected() {
        let x = noise(11, TEST_LEN);
        let flat = vec![5.0f32; TEST_LEN];
        let e = estimate_coherence(&flat, &x, TEST_SAMPLE_RATE).unwrap_err();
        assert!(e.contains("setpoint is constant"), "{e}");
        let e = estimate_coherence(&x, &flat, TEST_SAMPLE_RATE).unwrap_err();
        assert!(e.contains("gyro is constant"), "{e}");
    }

    // Segment count boundary: 15 segments fail, 16 pass (minimum 16), from the segment formula
    // (len - seg) / hop + 1 with seg 512, hop 256.
    #[test]
    fn minimum_segment_count_boundary() {
        let seg = 512;
        let len_15 = 14 * 256 + seg;
        let len_16 = 15 * 256 + seg;
        assert_eq!(segment_count_for(len_15, seg), 15);
        assert_eq!(segment_count_for(len_16, seg), 16);
        let x = noise(12, len_16);
        assert!(estimate_coherence(&x[..len_15], &x[..len_15], TEST_SAMPLE_RATE).is_err());
        assert!(estimate_coherence(&x, &x, TEST_SAMPLE_RATE).is_ok());
        assert_eq!(segment_count_for(seg - 1, seg), 0);
    }

    // Signals of 0.0577 deg/s std pass the constant check but give Sxx * Syy near 1e-17, far
    // below the 1e-12 guard of the shared estimate: every bin is undefined, so the call errors
    // instead of returning a curve of zeros.
    #[test]
    fn signals_without_power_have_no_defined_bin() {
        let quiet: Vec<f32> = noise(17, TEST_LEN)
            .iter()
            .map(|v| v / TEST_AMPLITUDE_DEG_S as f32 * 0.1)
            .collect();
        let e = estimate_coherence(&quiet, &quiet, TEST_SAMPLE_RATE).unwrap_err();
        assert!(e.contains("no frequency bin"), "{e}");
    }

    #[test]
    fn invalid_sample_rate_and_length_mismatch_are_rejected() {
        let x = noise(13, TEST_LEN);
        assert!(estimate_coherence(&x, &x, 0.0).is_err());
        assert!(estimate_coherence(&x, &x, f64::NAN).is_err());
        assert!(estimate_coherence(&x, &x[1..], TEST_SAMPLE_RATE).is_err());
    }

    #[test]
    fn segment_length_is_power_of_two_for_common_rates() {
        assert_eq!(segment_length_for(1000.0), 512);
        assert_eq!(segment_length_for(2000.0), 1024);
        assert_eq!(segment_length_for(4000.0), 2048);
        assert_eq!(segment_length_for(8000.0), 4096);
    }

    // A DC offset must not leak into the result: the same signals plus a large offset
    // gives the same coherence as without.
    #[test]
    fn dc_offset_does_not_change_coherence() {
        let x = noise(14, TEST_LEN);
        let n = noise(15, TEST_LEN);
        let y: Vec<f32> = x.iter().zip(&n).map(|(a, b)| a + b).collect();
        let y_off: Vec<f32> = y.iter().map(|v| v + 100.0).collect();
        let (_, a, _) = estimate_coherence(&x, &y, TEST_SAMPLE_RATE).expect("ok");
        let (_, b, _) = estimate_coherence(&x, &y_off, TEST_SAMPLE_RATE).expect("ok");
        assert!((mean(&a) - mean(&b)).abs() < 1e-3);
    }

    #[test]
    fn bandwidth_is_the_unbroken_run_above_the_medium_threshold() {
        // bins 0..5 at 0, 10, ..., 50 Hz; DC is skipped, the run breaks at 40 Hz
        let freq = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0];
        let coh = [0.0, 0.9, 0.4, 0.39, 0.95, 0.95];
        assert_eq!(coherent_bandwidth(&freq, &coh), Some(20.0));
        assert_eq!(
            coherent_bandwidth(&freq, &[0.0, 0.1, 0.9, 0.9, 0.9, 0.9]),
            None
        );
    }

    #[test]
    fn mean_band_excludes_bins_above_the_limit() {
        let freq = [0.0, 50.0, 100.0, 150.0];
        let coh = [0.2, 0.2, 0.8, 1.0];
        let (m, lo, hi) = mean_in_band(&freq, &coh).expect("mean");
        assert!((m - 0.4).abs() < 1e-12, "{m}");
        assert_eq!((lo, hi), (0.0, 100.0));
        assert!(mean_in_band(&[150.0], &[1.0]).is_none());
    }

    #[test]
    fn analyze_axis_pairs_rows_and_skips_missing_axis() {
        let sine: Vec<f64> = (0..TEST_LEN)
            .map(|i| {
                (2.0 * PI * 20.0 * i as f64 / TEST_SAMPLE_RATE).sin() * 10.0 * TEST_AMPLITUDE_DEG_S
            })
            .collect();
        let rows: Vec<LogRowData> = sine
            .iter()
            .zip(noise(16, TEST_LEN))
            .map(|(&s, n)| LogRowData {
                setpoint: [Some(s), None, Some(1.0), None],
                gyro: [Some(s + n as f64), None, Some(1.0)],
                ..Default::default()
            })
            .collect();
        let r = analyze_axis(&rows, TEST_SAMPLE_RATE, 0).expect("roll");
        assert_eq!(r.axis_name, "Roll");
        // A 20 Hz sine of 1000 deg/s against noise of std ~58 deg/s: the sine bin holds nearly all power.
        let bin = r
            .frequency_hz
            .iter()
            .position(|&f| (f - 20.0).abs() < 1.0)
            .expect("bin");
        assert!(r.coherence[bin] > 0.99, "{}", r.coherence[bin]);
        assert!(analyze_axis(&rows, TEST_SAMPLE_RATE, 1).is_err());
        assert!(analyze_axis(&rows, TEST_SAMPLE_RATE, 2).is_err());
        assert!(analyze_axis(&rows, TEST_SAMPLE_RATE, 3).is_err());
        assert!(analyze_coherence(&rows, None).is_empty());
        assert_eq!(analyze_coherence(&rows, Some(TEST_SAMPLE_RATE)).len(), 1);
    }

    #[test]
    fn summary_text_is_shared_by_console_lines() {
        let r = CoherenceResult {
            axis_name: "Roll".to_string(),
            frequency_hz: vec![],
            coherence: vec![],
            segment_count: 20,
            mean_coherence: Some((0.5, 2.0, 98.0)),
            coherent_bandwidth_hz: None,
        };
        let t = summary_text(&r);
        assert_eq!(t, "mean 0.50 over 2.0-98.0 Hz, coherence >= 0.4 up to none");
        assert!(console_lines(&[r])[0].contains(&t));
    }
}
