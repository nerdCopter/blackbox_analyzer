// src/data_analysis/filter_group_delay.rs
//
// Theoretical group delay tau(f) = -d(phi)/d(omega) of the firmware lowpass chain, built from the
// filter settings in the log header. Not measured from flight data.

use crate::axis_names::AXIS_NAMES;
use crate::constants::{
    BIQUAD_Q_MAX, BIQUAD_Q_MIN, GROUP_DELAY_PLOT_MAX_HZ, GROUP_DELAY_PLOT_POINTS,
    GROUP_DELAY_REPORT_FREQS_HZ, SECONDS_TO_MS,
};
use crate::data_analysis::filter_response::{
    calculate_ptn_per_stage_cutoff, AllFilterConfigs, AxisFilterConfig, FilterType,
};

pub const SIGNAL_GYRO: &str = "Gyro";
pub const SIGNAL_DTERM: &str = "D-term";

/// One lowpass stage of a chain: a filter type at one cutoff (and Q for a biquad).
#[derive(Debug, Clone, PartialEq)]
pub struct DelayStage {
    pub label: String,
    pub filter_type: FilterType,
    pub cutoff_hz: f64,
    pub q_factor: Option<f64>,
}

/// Lowpass chain of one signal on one axis. `stages` use the dynamic LPF1 minimum cutoff (largest
/// delay); `stages_at_max_cutoff` is set only when LPF1 is dynamic with a larger maximum cutoff.
#[derive(Debug, Clone, PartialEq)]
pub struct ChainDelay {
    pub signal: &'static str,
    pub stages: Vec<DelayStage>,
    pub stages_at_max_cutoff: Option<Vec<DelayStage>>,
}

/// Per-axis group delay chains for the gyro and D-term signals.
#[derive(Debug, Clone, PartialEq)]
pub struct AxisGroupDelay {
    pub axis_name: String,
    pub gyro: Option<ChainDelay>,
    pub dterm: Option<ChainDelay>,
}

/// Number of cascaded PT1 stages in a firmware PTn filter, or None for a biquad.
fn pt_stage_count(filter_type: FilterType) -> Option<u32> {
    match filter_type {
        FilterType::PT1 => Some(1),
        FilterType::PT2 => Some(2),
        FilterType::PT3 => Some(3),
        FilterType::PT4 => Some(4),
        FilterType::Biquad => None,
    }
}

/// Group delay (seconds) of one first-order lowpass: H(s) = 1 / (1 + s/w0).
/// phi = -atan(w/w0), so tau = 1 / (w0 * (1 + (w/w0)^2)); tau(0) = 1 / w0.
fn pt1_group_delay_s(frequency_hz: f64, cutoff_hz: f64) -> f64 {
    let omega_0 = 2.0 * std::f64::consts::PI * cutoff_hz;
    let ratio = frequency_hz / cutoff_hz;
    1.0 / (omega_0 * (1.0 + ratio * ratio))
}

/// Group delay (seconds) of a second-order lowpass H(s) = w0^2 / (s^2 + (w0/Q) s + w0^2).
/// phi = -atan2(r/Q, 1 - r^2) with r = w/w0, so
/// tau = (1 / (w0 Q)) * (1 + r^2) / ((1 - r^2)^2 + (r/Q)^2); tau(0) = 1 / (w0 Q).
/// The closed form needs no phase unwrapping, so it stays continuous through the -180 degree wrap.
fn biquad_group_delay_s(frequency_hz: f64, cutoff_hz: f64, q_factor: f64) -> f64 {
    let omega_0 = 2.0 * std::f64::consts::PI * cutoff_hz;
    let ratio = frequency_hz / cutoff_hz;
    let ratio_sq = ratio * ratio;
    let denominator = (1.0 - ratio_sq).powi(2) + (ratio / q_factor).powi(2);
    (1.0 + ratio_sq) / (omega_0 * q_factor * denominator)
}

/// Group delay (seconds) of one filter stage at `frequency_hz`.
///
/// PT1..PT4 are cascades of PT1 stages whose cutoff is the configured cutoff times the PTn
/// correction (`calculate_ptn_per_stage_cutoff`), as in the firmware filter code. A biquad uses
/// the same Q clamp and Butterworth default as `biquad_response_with_q`. A cutoff of zero or less
/// (or NaN) means no filtering (zero delay), matching the magnitude functions; a non-finite
/// frequency gives NaN. The result is even in
/// frequency, so a negative frequency uses its absolute value; frequency 0 gives the DC delay.
pub fn stage_group_delay_s(
    frequency_hz: f64,
    filter_type: FilterType,
    cutoff_hz: f64,
    q_factor: Option<f64>,
) -> f64 {
    if cutoff_hz.is_nan() || cutoff_hz <= 0.0 {
        return 0.0;
    }
    if !frequency_hz.is_finite() {
        return f64::NAN;
    }
    let frequency_hz = frequency_hz.abs();
    match pt_stage_count(filter_type) {
        Some(stages) => {
            let per_stage_cutoff_hz = calculate_ptn_per_stage_cutoff(cutoff_hz, stages);
            f64::from(stages) * pt1_group_delay_s(frequency_hz, per_stage_cutoff_hz)
        }
        None => {
            let q = q_factor
                .unwrap_or(std::f64::consts::FRAC_1_SQRT_2)
                .clamp(BIQUAD_Q_MIN, BIQUAD_Q_MAX);
            biquad_group_delay_s(frequency_hz, cutoff_hz, q)
        }
    }
}

/// Group delay (seconds) of a cascade: delays of the stages add.
pub fn chain_group_delay_s(frequency_hz: f64, stages: &[DelayStage]) -> f64 {
    stages
        .iter()
        .map(|s| stage_group_delay_s(frequency_hz, s.filter_type, s.cutoff_hz, s.q_factor))
        .sum()
}

/// Group delay in milliseconds, the unit shown in every output.
pub fn chain_group_delay_ms(frequency_hz: f64, stages: &[DelayStage]) -> f64 {
    chain_group_delay_s(frequency_hz, stages) * SECONDS_TO_MS
}

fn stage_from_filter(
    label: &str,
    filter_type: FilterType,
    cutoff_hz: f64,
    q_factor: Option<f64>,
) -> DelayStage {
    DelayStage {
        label: format!("{label} {} {cutoff_hz:.0} Hz", filter_type.name()),
        filter_type,
        cutoff_hz,
        q_factor,
    }
}

/// Build the lowpass chain for one signal of one axis. Selection follows
/// `generate_individual_filter_curves`: a dynamic LPF1 replaces a static LPF1, then LPF2, then the
/// IMUF external PTn filter. The IMUF pseudo-Kalman filter (no PTn cutoff) is nonlinear and has no
/// delay model here. Notch and RPM filters are not part of the chain.
fn build_chain(signal: &'static str, axis: &AxisFilterConfig) -> Option<ChainDelay> {
    let mut stages = Vec::new();
    let mut stages_max = Vec::new();
    let mut has_max_variant = false;

    if let Some(ref dynamic) = axis.dynamic_lpf1 {
        if dynamic.enabled && dynamic.min_cutoff_hz > 0.0 {
            let min_stage =
                stage_from_filter("Dyn LPF1", dynamic.filter_type, dynamic.min_cutoff_hz, None);
            if dynamic.max_cutoff_hz > dynamic.min_cutoff_hz {
                has_max_variant = true;
                stages_max.push(stage_from_filter(
                    "Dyn LPF1",
                    dynamic.filter_type,
                    dynamic.max_cutoff_hz,
                    None,
                ));
            } else {
                stages_max.push(min_stage.clone());
            }
            stages.push(min_stage);
        }
    } else if let Some(ref lpf1) = axis.lpf1 {
        if lpf1.enabled && lpf1.cutoff_hz > 0.0 {
            let stage = stage_from_filter("LPF1", lpf1.filter_type, lpf1.cutoff_hz, lpf1.q_factor);
            stages_max.push(stage.clone());
            stages.push(stage);
        }
    }

    let mut push_both = |stage: DelayStage| {
        stages_max.push(stage.clone());
        stages.push(stage);
    };

    if let Some(ref lpf2) = axis.lpf2 {
        if lpf2.enabled && lpf2.cutoff_hz > 0.0 {
            push_both(stage_from_filter(
                "LPF2",
                lpf2.filter_type,
                lpf2.cutoff_hz,
                lpf2.q_factor,
            ));
        }
    }

    if let Some(ref imuf) = axis.imuf {
        if imuf.enabled && imuf.lowpass_cutoff_hz > 0.0 {
            let filter_type = match imuf.ptn_order {
                1 => FilterType::PT1,
                3 => FilterType::PT3,
                4 => FilterType::PT4,
                _ => FilterType::PT2,
            };
            push_both(stage_from_filter(
                "IMUF",
                filter_type,
                imuf.lowpass_cutoff_hz,
                None,
            ));
        }
    }

    if stages.is_empty() {
        return None;
    }
    Some(ChainDelay {
        signal,
        stages,
        stages_at_max_cutoff: has_max_variant.then_some(stages_max),
    })
}

/// Build the per-axis gyro and D-term group delay chains from the parsed header filter settings.
pub fn analyze_group_delay(configs: &AllFilterConfigs) -> Vec<AxisGroupDelay> {
    AXIS_NAMES
        .iter()
        .enumerate()
        .map(|(axis, name)| AxisGroupDelay {
            axis_name: (*name).to_string(),
            gyro: build_chain(SIGNAL_GYRO, &configs.gyro[axis]),
            dterm: build_chain(SIGNAL_DTERM, &configs.dterm[axis]),
        })
        .collect()
}

/// Group delay curve (frequency Hz, delay ms) on the plot grid, 0 Hz to `GROUP_DELAY_PLOT_MAX_HZ`.
pub fn group_delay_curve_ms(stages: &[DelayStage]) -> Vec<(f64, f64)> {
    let last = (GROUP_DELAY_PLOT_POINTS - 1) as f64;
    (0..GROUP_DELAY_PLOT_POINTS)
        .map(|i| {
            let f = GROUP_DELAY_PLOT_MAX_HZ * i as f64 / last;
            (f, chain_group_delay_ms(f, stages))
        })
        .collect()
}

/// Stage list text, e.g. "Dyn LPF1 PT1 75 Hz + LPF2 PT1 150 Hz".
pub fn stages_text(stages: &[DelayStage]) -> String {
    stages
        .iter()
        .map(|s| s.label.as_str())
        .collect::<Vec<_>>()
        .join(" + ")
}

/// Delays (ms) at the report frequencies as text. A dynamic LPF1 shows "min-cutoff to max-cutoff".
/// One function feeds the console, the report and the plot legend.
pub fn delays_text(chain: &ChainDelay) -> String {
    GROUP_DELAY_REPORT_FREQS_HZ
        .iter()
        .map(|&f| {
            let slow = chain_group_delay_ms(f, &chain.stages);
            let value = match chain.stages_at_max_cutoff {
                Some(ref fast) => format!("{slow:.2} to {:.2}", chain_group_delay_ms(f, fast)),
                None => format!("{slow:.2}"),
            };
            if f == 0.0 {
                format!("DC {value} ms")
            } else {
                format!("{f:.0} Hz {value} ms")
            }
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// Console lines, one per axis and signal with a chain.
pub fn console_lines(results: &[AxisGroupDelay]) -> Vec<String> {
    let mut lines = Vec::new();
    for r in results {
        for chain in [&r.gyro, &r.dterm].into_iter().flatten() {
            lines.push(format!(
                "{} {} filter group delay: {} ({})",
                r.axis_name,
                chain.signal,
                delays_text(chain),
                stages_text(&chain.stages)
            ));
        }
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_analysis::filter_response::{DynamicFilterConfig, FilterConfig};
    use std::f64::consts::PI;

    const TOL: f64 = 1e-12;

    fn stage(filter_type: FilterType, cutoff_hz: f64, q: Option<f64>) -> DelayStage {
        stage_from_filter("S", filter_type, cutoff_hz, q)
    }

    /// Analytic PT1 delay from the definition: 1 / (w0 (1 + (w/w0)^2)).
    fn analytic_pt1(f: f64, fc: f64) -> f64 {
        let w0 = 2.0 * PI * fc;
        let w = 2.0 * PI * f;
        1.0 / (w0 * (1.0 + (w / w0).powi(2)))
    }

    /// Complex response of a chain, independent of the closed forms under test.
    fn complex_h(f: f64, stages: &[DelayStage]) -> (f64, f64) {
        let w = 2.0 * PI * f;
        let mut re = 1.0;
        let mut im = 0.0;
        for s in stages {
            let w0 = 2.0 * PI * s.cutoff_hz;
            let (sr, si) = match pt_stage_count(s.filter_type) {
                Some(n) => {
                    let wc = 2.0 * PI * calculate_ptn_per_stage_cutoff(s.cutoff_hz, n);
                    // (1 / (1 + jw/wc))^n
                    let mut hr = 1.0;
                    let mut hi = 0.0;
                    for _ in 0..n {
                        let dr = 1.0;
                        let di = w / wc;
                        let m = dr * dr + di * di;
                        let (pr, pi) = (dr / m, -di / m);
                        let nr = hr * pr - hi * pi;
                        let ni = hr * pi + hi * pr;
                        hr = nr;
                        hi = ni;
                    }
                    (hr, hi)
                }
                None => {
                    let q = s.q_factor.unwrap_or(std::f64::consts::FRAC_1_SQRT_2);
                    let r = w / w0;
                    let dr = 1.0 - r * r;
                    let di = r / q;
                    let m = dr * dr + di * di;
                    (dr / m, -di / m)
                }
            };
            let nr = re * sr - im * si;
            let ni = re * si + im * sr;
            re = nr;
            im = ni;
        }
        (re, im)
    }

    /// Group delay by central difference of the phase unwrapped from 0 Hz upward in 0.01 Hz steps.
    fn numeric_delay_s(stages: &[DelayStage], f: f64) -> f64 {
        const STEP_HZ: f64 = 0.01;
        const HALF_SPAN_STEPS: i64 = 5;
        let center = (f / STEP_HZ).round() as i64;
        let mut offset = 0.0;
        let mut prev_raw = 0.0;
        let mut at_lo = 0.0;
        let mut at_hi = 0.0;
        for k in 0..=(center + HALF_SPAN_STEPS) {
            let (re, im) = complex_h(k as f64 * STEP_HZ, stages);
            let raw = im.atan2(re);
            if k > 0 {
                if raw - prev_raw > PI {
                    offset -= 2.0 * PI;
                } else if raw - prev_raw < -PI {
                    offset += 2.0 * PI;
                }
            }
            prev_raw = raw;
            if k == center - HALF_SPAN_STEPS {
                at_lo = raw + offset;
            }
            if k == center + HALF_SPAN_STEPS {
                at_hi = raw + offset;
            }
        }
        -(at_hi - at_lo) / (2.0 * PI * 2.0 * HALF_SPAN_STEPS as f64 * STEP_HZ)
    }

    #[test]
    fn pt1_matches_analytic_definition() {
        for &(f, fc) in &[(0.0, 100.0), (50.0, 100.0), (100.0, 100.0), (400.0, 100.0)] {
            let got = stage_group_delay_s(f, FilterType::PT1, fc, None);
            assert!((got - analytic_pt1(f, fc)).abs() < TOL, "f={f} fc={fc}");
        }
    }

    #[test]
    fn pt1_dc_delay_is_one_over_w0() {
        let fc = 80.0;
        let got = stage_group_delay_s(0.0, FilterType::PT1, fc, None);
        assert!((got - 1.0 / (2.0 * PI * fc)).abs() < TOL);
    }

    #[test]
    fn pt1_delay_at_cutoff_is_half_dc_delay() {
        let at_cutoff = stage_group_delay_s(100.0, FilterType::PT1, 100.0, None);
        let dc = stage_group_delay_s(0.0, FilterType::PT1, 100.0, None);
        assert!((at_cutoff - dc / 2.0).abs() < TOL);
    }

    #[test]
    fn ptn_is_n_pt1_stages_at_corrected_cutoff() {
        // Corrections from the firmware: PT2 1.553773974, PT3 1.961459177, PT4 2.298959223.
        for &(ft, n, corr) in &[
            (FilterType::PT2, 2.0, 1.553773974),
            (FilterType::PT3, 3.0, 1.961459177),
            (FilterType::PT4, 4.0, 2.298959223),
        ] {
            for &f in &[0.0, 60.0, 200.0] {
                let got = stage_group_delay_s(f, ft, 90.0, None);
                let want = n * analytic_pt1(f, 90.0 * corr);
                assert!((got - want).abs() < 1e-9, "{ft:?} f={f}");
            }
        }
    }

    #[test]
    fn biquad_dc_delay_is_one_over_w0_q() {
        let fc = 120.0;
        for &q in &[0.5, std::f64::consts::FRAC_1_SQRT_2, 2.0] {
            let got = stage_group_delay_s(0.0, FilterType::Biquad, fc, Some(q));
            assert!((got - 1.0 / (2.0 * PI * fc * q)).abs() < TOL, "q={q}");
        }
    }

    #[test]
    fn biquad_default_q_is_butterworth() {
        let a = stage_group_delay_s(80.0, FilterType::Biquad, 100.0, None);
        let b = stage_group_delay_s(
            80.0,
            FilterType::Biquad,
            100.0,
            Some(std::f64::consts::FRAC_1_SQRT_2),
        );
        assert!((a - b).abs() < TOL);
    }

    #[test]
    fn cascade_sums_stage_delays() {
        let a = stage(FilterType::PT1, 120.0, None);
        let b = stage(FilterType::Biquad, 250.0, Some(0.9));
        let c = stage(FilterType::PT3, 90.0, None);
        for &f in &[0.0, 75.0, 333.0] {
            let sum = chain_group_delay_s(f, &[a.clone(), b.clone(), c.clone()]);
            let parts = chain_group_delay_s(f, std::slice::from_ref(&a))
                + chain_group_delay_s(f, std::slice::from_ref(&b))
                + chain_group_delay_s(f, std::slice::from_ref(&c));
            assert!((sum - parts).abs() < TOL);
        }
        assert_eq!(chain_group_delay_s(10.0, &[]), 0.0);
    }

    #[test]
    fn matches_numeric_derivative_of_unwrapped_phase() {
        // A biquad plus PT3 passes through well more than 180 degrees, so the raw atan2 phase
        // wraps; the unwrapped numeric derivative must still equal the closed form.
        let chain = [
            stage(FilterType::Biquad, 100.0, Some(3.0)),
            stage(FilterType::PT3, 120.0, None),
            stage(FilterType::PT1, 200.0, None),
        ];
        let mut wrapped = false;
        let mut last_raw = 0.0;
        for k in 1..=500 {
            let (re, im) = complex_h(f64::from(k), &chain);
            let raw = im.atan2(re);
            if (raw - last_raw).abs() > PI {
                wrapped = true;
            }
            last_raw = raw;
        }
        assert!(wrapped, "test chain must cross the +-180 degree wrap");
        for &f in &[10.0, 60.0, 100.0, 140.0, 250.0] {
            let analytic = chain_group_delay_s(f, &chain);
            let numeric = numeric_delay_s(&chain, f);
            assert!(
                (analytic - numeric).abs() < 1e-5 * analytic.abs(),
                "f={f} analytic={analytic} numeric={numeric}"
            );
        }
    }

    #[test]
    fn zero_or_negative_cutoff_is_no_filtering() {
        assert_eq!(stage_group_delay_s(50.0, FilterType::PT2, 0.0, None), 0.0);
        assert_eq!(
            stage_group_delay_s(50.0, FilterType::Biquad, -5.0, None),
            0.0
        );
        assert_eq!(
            stage_group_delay_s(50.0, FilterType::PT1, f64::NAN, None),
            0.0
        );
        assert!(stage_group_delay_s(f64::NAN, FilterType::PT1, 100.0, None).is_nan());
    }

    #[test]
    fn frequency_zero_and_sign_are_handled() {
        let dc = stage_group_delay_s(0.0, FilterType::PT2, 100.0, None);
        assert!(dc.is_finite() && dc > 0.0);
        let neg = stage_group_delay_s(-80.0, FilterType::PT2, 100.0, None);
        let pos = stage_group_delay_s(80.0, FilterType::PT2, 100.0, None);
        assert!((neg - pos).abs() < TOL);
        assert!(stage_group_delay_s(f64::INFINITY, FilterType::PT1, 100.0, None).is_nan());
    }

    #[test]
    fn biquad_q_is_clamped_like_the_magnitude_model() {
        let low = stage_group_delay_s(50.0, FilterType::Biquad, 100.0, Some(0.01));
        let at_min = stage_group_delay_s(50.0, FilterType::Biquad, 100.0, Some(BIQUAD_Q_MIN));
        assert!((low - at_min).abs() < TOL);
        let high = stage_group_delay_s(50.0, FilterType::Biquad, 100.0, Some(500.0));
        let at_max = stage_group_delay_s(50.0, FilterType::Biquad, 100.0, Some(BIQUAD_Q_MAX));
        assert!((high - at_max).abs() < TOL);
    }

    fn lpf(filter_type: FilterType, cutoff_hz: f64) -> FilterConfig {
        FilterConfig {
            filter_type,
            cutoff_hz,
            q_factor: None,
            enabled: true,
        }
    }

    #[test]
    fn chain_selection_follows_overlay_rules() {
        let mut axis = AxisFilterConfig {
            lpf1: Some(lpf(FilterType::PT1, 250.0)),
            lpf2: Some(lpf(FilterType::PT1, 500.0)),
            dynamic_lpf1: Some(DynamicFilterConfig {
                filter_type: FilterType::PT1,
                min_cutoff_hz: 200.0,
                max_cutoff_hz: 400.0,
                expo: 5,
                enabled: true,
            }),
            imuf: None,
        };
        let chain = build_chain(SIGNAL_GYRO, &axis).expect("chain");
        // Dynamic LPF1 replaces the static LPF1; LPF2 stays.
        assert_eq!(chain.stages.len(), 2);
        assert_eq!(chain.stages[0].cutoff_hz, 200.0);
        let fast = chain.stages_at_max_cutoff.as_ref().expect("max variant");
        assert_eq!(fast[0].cutoff_hz, 400.0);
        assert_eq!(fast[1].cutoff_hz, 500.0);
        // The higher cutoff delays less at DC.
        assert!(chain_group_delay_s(0.0, fast) < chain_group_delay_s(0.0, &chain.stages));

        // Dynamic with equal ends: no separate max variant.
        axis.dynamic_lpf1.as_mut().unwrap().max_cutoff_hz = 200.0;
        assert!(build_chain(SIGNAL_GYRO, &axis)
            .unwrap()
            .stages_at_max_cutoff
            .is_none());

        // No enabled stage: no chain.
        assert!(build_chain(SIGNAL_DTERM, &AxisFilterConfig::default()).is_none());
    }

    #[test]
    fn imuf_without_cutoff_adds_no_stage() {
        use crate::data_analysis::filter_response::ImufFilterConfig;
        let axis = AxisFilterConfig {
            imuf: Some(ImufFilterConfig {
                lowpass_cutoff_hz: 0.0,
                ptn_order: 2,
                q_factor: 6.0,
                revision: None,
                pseudo_kalman_w: Some(32.0),
                effective_cutoff_hz: 0.0,
                enabled: true,
            }),
            ..Default::default()
        };
        assert!(build_chain(SIGNAL_GYRO, &axis).is_none());
    }

    #[test]
    fn shared_text_has_dc_and_report_frequencies() {
        let chain = ChainDelay {
            signal: SIGNAL_GYRO,
            stages: vec![stage(FilterType::PT1, 100.0, None)],
            stages_at_max_cutoff: None,
        };
        let text = delays_text(&chain);
        let dc_ms = 1.0 / (2.0 * PI * 100.0) * SECONDS_TO_MS;
        assert!(text.starts_with(&format!("DC {dc_ms:.2} ms")), "{text}");
        assert!(text.contains("100 Hz") && text.contains("200 Hz"));
    }

    #[test]
    fn curve_grid_spans_zero_to_max() {
        let curve = group_delay_curve_ms(&[stage(FilterType::PT1, 100.0, None)]);
        assert_eq!(curve.len(), GROUP_DELAY_PLOT_POINTS);
        assert_eq!(curve[0].0, 0.0);
        assert!((curve.last().unwrap().0 - GROUP_DELAY_PLOT_MAX_HZ).abs() < TOL);
        assert!(
            curve.windows(2).all(|w| w[1].1 <= w[0].1),
            "PT1 delay falls with frequency"
        );
    }
}
