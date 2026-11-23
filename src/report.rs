// src/report.rs
// Structured markdown report capturing computed flight analysis outputs.
// Derives all content from typed structs returned by analysis functions —
// no re-reading of CSV data, no duplication of println logic.

use std::error::Error;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use crate::axis_names::{AXIS_COUNT, AXIS_NAMES};
use crate::constants::VIBRATION_PERCENTILES_PCT;
use crate::constants::{
    MOTOR_DESYNC_REPORT_MAX_TIMES, MOTOR_OSCILLATION_FREQ_MAX_HZ, MOTOR_OSCILLATION_FREQ_MIN_HZ,
    MOTOR_OSCILLATION_SECONDS_TO_MS, MOTOR_OSCILLATION_WINDOW_S, RATIO_TO_PERCENT,
    STICK_DIST_FULL_STICK_RC_COMMAND,
};
use crate::data_analysis::filter_response::{
    rpm_motor_frequency_lines, AllFilterConfigs, DynamicNotchConfig, RpmFilterConfig,
    RpmMotorFrequencies,
};
use crate::data_analysis::optimal_p_estimation::{OptimalPAnalysis, PRecommendation};
use crate::data_analysis::stick_distribution::StickDistributionResult;
use crate::data_analysis::transfer_function_estimation::Confidence;
use crate::data_analysis::vibration_percentile::{
    format_percentile_value, VibrationPercentileResult,
};
use crate::data_input::pid_metadata::format_d_target;
use crate::plot_functions::motor_desync::{
    fallback_oscillation_overlaps, DesyncConfidence, MotorDesyncResult,
};
use crate::plot_functions::plot_bode::BodeAxisResult;
use crate::plot_functions::plot_d_term_spectrums::DTermAxisResult;
use crate::plot_functions::plot_gyro_spectrums::GyroAnalysisResult;
use crate::plot_functions::plot_motor_spectrums::MotorOscillationResult;
use crate::plot_functions::plot_rc_command_activity::RcCommandStepResult;

/// D-term recommendation for one tier (conservative, moderate, or aggressive)
pub struct DTermRec {
    pub pd_ratio: f64,
    pub d: Option<u32>,
    pub d_min: Option<u32>,
    pub d_max: Option<u32>,
}

/// Step response analysis result for one axis
pub struct StepAxisReport {
    pub axis_name: &'static str,
    pub peak_value: f64,
    pub assessment: &'static str,
    pub current_pd_ratio: f64,
    pub conservative: Option<DTermRec>,
    pub moderate: Option<DTermRec>,
    pub aggressive: Option<DTermRec>,
    /// Optional D-1 hint shown for the near-optimal zone, which has no P:D recommendation.
    pub near_optimal_hint: Option<String>,
    pub setpoint_authority_name: Option<&'static str>,
    pub setpoint_authority_mean: Option<f32>,
    pub warnings: Vec<String>,
}

/// Aggregated analysis results collected from one flight log
pub struct FlightReport {
    pub root_name: String,
    pub sample_rate: Option<f64>,
    /// (start_s, end_s, row_count) when --start/--end trimmed the log, relative to log start.
    pub trim_window: Option<(f64, f64, usize)>,
    pub header_metadata: Vec<(String, String)>,
    pub pd_ratios: [Option<f64>; AXIS_COUNT],
    /// Per-axis gain text (e.g. "P:57 I:66 D:58 FF:0"); empty when the header has no PID data.
    pub pid_labels: [String; AXIS_COUNT],
    /// Per-axis D-Min and D-Max (`AxisPid::d_range_for_display`); `None` shows base D.
    pub d_ranges: [Option<(u32, u32)>; AXIS_COUNT],
    pub step_reports: Vec<StepAxisReport>,
    pub optimal_p: [Option<OptimalPAnalysis>; AXIS_COUNT],
    /// Per-axis reason Optimal P was skipped (same text as the console and PNG legend).
    pub optimal_p_skip: [Option<String>; AXIS_COUNT],
    pub gyro_analysis: Option<GyroAnalysisResult>,
    pub dterm_results: Vec<DTermAxisResult>,
    pub bode_results: Vec<BodeAxisResult>,
    pub motor_results: Vec<MotorOscillationResult>,
    pub motor_desync_results: Vec<MotorDesyncResult>,
    pub rc_command_steps: Vec<RcCommandStepResult>,
    pub stick_distribution_results: Vec<StickDistributionResult>,
    /// Empty unless the Gyro Vibration Percentile plot is enabled.
    pub vibration_percentile_results: Vec<VibrationPercentileResult>,
    pub png_links: Vec<String>,
    /// Human-readable labels of plot types that were enabled but produced no plottable data.
    pub skipped_plots: Vec<String>,
    pub filter_config: Option<AllFilterConfigs>,
    pub dynamic_notch: Option<DynamicNotchConfig>,
    pub rpm_filter: Option<RpmFilterConfig>,
    /// Motor frequency ranges derived from eRPM and motor_poles; None when the log lacks either.
    pub rpm_motor_frequencies: Option<RpmMotorFrequencies>,
    /// True when gyroUnfilt came from debug channels instead of dedicated gyroUnfilt columns.
    pub debug_fallback: bool,
    pub debug_mode_name: Option<&'static str>,
}

/// Generate a structured markdown report and write it to `output_path`.
pub fn generate_markdown_report(
    report: &FlightReport,
    output_path: &Path,
) -> Result<(), Box<dyn Error>> {
    let mut md = String::new();

    writeln!(md, "# BlackBox Flight Report: {}", report.root_name)?;
    writeln!(md)?;

    // --- Metadata ---
    writeln!(md, "## Metadata")?;
    writeln!(md)?;
    match report.sample_rate {
        Some(sr) => writeln!(md, "- **Sample Rate:** {:.1} Hz", sr)?,
        None => writeln!(md, "- **Sample Rate:** Unknown")?,
    }
    if let Some((start, end, rows)) = report.trim_window {
        writeln!(
            md,
            "- **Time Window:** {:.3}s – {:.3}s ({} rows)",
            start, end, rows
        )?;
    }
    let interesting_keys = [
        "Firmware revision",
        "Craft name",
        "looptime",
        "gyro_lpf_hz",
        "dterm_lpf_hz",
        "pid_process_denom",
        "rollPID",
        "pitchPID",
        "yawPID",
    ];
    for (k, v) in &report.header_metadata {
        if interesting_keys.iter().any(|ik| k.eq_ignore_ascii_case(ik)) {
            writeln!(md, "- **{}:** {}", k, v)?;
        }
    }
    if report.debug_fallback {
        let mode_str = report
            .debug_mode_name
            .map_or(String::new(), |m| format!(" ({})", m));
        writeln!(
            md,
            "- **⚠ gyroUnfilt source:** debug[0-2] fallback{} — unfiltered gyro spectrum and filtering delay derived from debug channels, not dedicated gyroUnfilt columns",
            mode_str
        )?;
    }
    writeln!(md)?;

    // --- Gyro Filter Configuration ---
    let mut filter_section_written = false;
    if let Some(ref fc) = report.filter_config {
        let axis_labels = ["Roll", "Pitch", "Yaw"];
        // Only emit section if at least one axis has any filter configured
        let has_any = (0..3).any(|i| {
            fc.gyro[i].lpf1.is_some()
                || fc.gyro[i].lpf2.is_some()
                || fc.gyro[i].dynamic_lpf1.is_some()
                || fc.gyro[i].imuf.is_some()
        });
        // IMUF / Pseudo-Kalman is an EmuFlight-only concept; only add the column
        // when at least one axis actually has it configured (i.e. EmuFlight logs).
        let has_imuf = (0..3).any(|i| fc.gyro[i].imuf.is_some());
        if has_any {
            filter_section_written = true;
            writeln!(md, "## Gyro Filter Configuration")?;
            writeln!(md)?;
            if has_imuf {
                writeln!(md, "| Axis | LPF1 | LPF2 | IMUF / Pseudo-Kalman |")?;
                writeln!(md, "|------|------|------|----------------------|")?;
            } else {
                writeln!(md, "| Axis | LPF1 | LPF2 |")?;
                writeln!(md, "|------|------|------|")?;
            }
            for (i, label) in axis_labels.iter().enumerate() {
                let lpf1 =
                    fmt_filter_stage(fc.gyro[i].lpf1.as_ref(), fc.gyro[i].dynamic_lpf1.as_ref());
                let lpf2 = fc.gyro[i]
                    .lpf2
                    .as_ref()
                    .map_or("—".into(), fmt_static_filter);
                if has_imuf {
                    let imuf = fc.gyro[i].imuf.as_ref().map_or("—".into(), fmt_imuf);
                    writeln!(md, "| {} | {} | {} | {} |", label, lpf1, lpf2, imuf)?;
                } else {
                    writeln!(md, "| {} | {} | {} |", label, lpf1, lpf2)?;
                }
            }
            writeln!(md)?;

            if let Some(ref dn) = report.dynamic_notch {
                writeln!(
                    md,
                    "- **Dynamic Notch:** {:.0}–{:.0} Hz, Q={:.0}, {} notch{}",
                    dn.min_hz,
                    dn.max_hz,
                    dn.q_factor,
                    dn.notch_count,
                    if dn.notch_count == 1 { "" } else { "es" }
                )?;
            }
            if let Some(ref rpm) = report.rpm_filter {
                write_rpm_filter_lines(&mut md, rpm, report.rpm_motor_frequencies.as_ref())?;
            }
            if report.dynamic_notch.is_some() || report.rpm_filter.is_some() {
                writeln!(md)?;
            }
        }
    }

    // An RPM filter is reported even when no gyro lowpass or IMUF stage is configured.
    if !filter_section_written {
        if let Some(ref rpm) = report.rpm_filter {
            writeln!(md, "## Gyro Filter Configuration")?;
            writeln!(md)?;
            write_rpm_filter_lines(&mut md, rpm, report.rpm_motor_frequencies.as_ref())?;
            writeln!(md)?;
        }
    }

    // --- PID Tuning ---
    writeln!(md, "## PID Tuning")?;
    writeln!(md)?;
    writeln!(md, "| Axis | PID | P:D Ratio |")?;
    writeln!(md, "|------|-----|-----------|")?;
    let axis_labels = ["Roll", "Pitch", "Yaw (informational)"];
    for (i, label) in axis_labels.iter().enumerate() {
        let pid = if report.pid_labels[i].is_empty() {
            "N/A"
        } else {
            report.pid_labels[i].as_str()
        };
        match report.pd_ratios[i] {
            Some(r) => writeln!(md, "| {} | {} | {:.2} |", label, pid, r)?,
            None => writeln!(md, "| {} | {} | N/A |", label, pid)?,
        }
    }
    writeln!(md)?;

    // --- Step Response Analysis ---
    if !report.step_reports.is_empty() {
        writeln!(md, "## Step Response Analysis")?;
        writeln!(md)?;
        for axis_report in &report.step_reports {
            writeln!(md, "### {}", axis_report.axis_name)?;
            writeln!(md)?;
            writeln!(md, "- **Peak Value:** {:.3}", axis_report.peak_value)?;
            writeln!(md, "- **Assessment:** {}", axis_report.assessment)?;
            writeln!(md, "- **Current P:D:** {:.2}", axis_report.current_pd_ratio)?;
            if let (Some(name), Some(mean)) = (
                axis_report.setpoint_authority_name,
                axis_report.setpoint_authority_mean,
            ) {
                writeln!(
                    md,
                    "- **Setpoint Authority:** {} (mean {:.0} dps)",
                    name, mean
                )?;
            }
            if let Some(rec) = &axis_report.conservative {
                writeln!(
                    md,
                    "- **Recommendation (conservative):** {}",
                    fmt_dterm_rec(rec)
                )?;
            }
            if let Some(rec) = &axis_report.moderate {
                writeln!(
                    md,
                    "- **Recommendation (moderate):** {}",
                    fmt_dterm_rec(rec)
                )?;
            }
            if let Some(rec) = &axis_report.aggressive {
                writeln!(
                    md,
                    "- **Recommendation (aggressive):** {}",
                    fmt_dterm_rec(rec)
                )?;
            }
            if let Some(hint) = &axis_report.near_optimal_hint {
                writeln!(md, "- **Recommendation (conservative):** {}", hint)?;
            }
            if axis_report.conservative.is_none()
                && axis_report.moderate.is_none()
                && axis_report.aggressive.is_none()
                && axis_report.near_optimal_hint.is_none()
            {
                writeln!(
                    md,
                    "- **Recommendation:** No obvious tuning adjustments needed"
                )?;
            }
            for w in &axis_report.warnings {
                writeln!(md, "- **⚠ Warning:** {}", w)?;
            }
            writeln!(md)?;
        }
    }

    // --- Gyro Analysis (per-axis filtering delay + spectrum peaks) ---
    if let Some(gyro) = &report.gyro_analysis {
        let has_gyro_data = gyro
            .axes
            .iter()
            .any(|a| !a.peaks.is_empty() || a.delay_ms.is_some());
        if has_gyro_data {
            writeln!(md, "## Gyro Analysis")?;
            writeln!(md)?;
            writeln!(
                md,
                "| Axis | Delay (ms) | Confidence | Peak | Freq (Hz) | Amplitude |"
            )?;
            writeln!(
                md,
                "|------|-----------|------------|------|-----------|-----------|"
            )?;
            for axis in &gyro.axes {
                let delay = axis.delay_ms.map_or("N/A".into(), |v| format!("{:.2}", v));
                let conf = axis
                    .delay_confidence
                    .map_or("N/A".into(), |v| format!("{:.0}%", v * 100.0));
                if axis.peaks.is_empty() {
                    writeln!(
                        md,
                        "| {} | {} | {} | N/A | N/A | N/A |",
                        axis.axis_name, delay, conf
                    )?;
                } else {
                    for (idx, (freq, amp)) in axis.peaks.iter().enumerate() {
                        let label = if idx == 0 {
                            "Primary".to_string()
                        } else {
                            format!("Sub {}", idx)
                        };
                        let (d_col, c_col) = if idx == 0 {
                            (delay.clone(), conf.clone())
                        } else {
                            ("".into(), "".into())
                        };
                        writeln!(
                            md,
                            "| {} | {} | {} | {} | {:.1} | {:.2} |",
                            axis.axis_name, d_col, c_col, label, freq, amp
                        )?;
                    }
                }
            }
            writeln!(md)?;
        }
    }

    // --- D-Term Analysis (per-axis delay + spectrum peaks) ---
    let has_dterm_data = report
        .dterm_results
        .iter()
        .any(|r| !r.peaks.is_empty() || r.delay_ms.is_some());
    if has_dterm_data {
        writeln!(md, "## D-Term Analysis")?;
        writeln!(md)?;
        writeln!(
            md,
            "| Axis | Delay (ms) | Confidence | Peak | Freq (Hz) | Amplitude |"
        )?;
        writeln!(
            md,
            "|------|-----------|------------|------|-----------|-----------|"
        )?;
        for r in &report.dterm_results {
            let delay = r.delay_ms.map_or_else(
                || {
                    r.na_reason
                        .map_or("N/A".into(), |reason| format!("N/A ({})", reason))
                },
                |v| format!("{:.1}", v),
            );
            let conf = r
                .delay_confidence
                .map_or("N/A".into(), |v| format!("{:.0}%", v * 100.0));
            if r.peaks.is_empty() {
                writeln!(
                    md,
                    "| {} | {} | {} | N/A | N/A | N/A |",
                    r.axis_name, delay, conf
                )?;
            } else {
                for (idx, (freq, amp)) in r.peaks.iter().enumerate() {
                    let label = if idx == 0 {
                        "Primary".to_string()
                    } else {
                        format!("Sub {}", idx)
                    };
                    let (d_col, c_col) = if idx == 0 {
                        (delay.clone(), conf.clone())
                    } else {
                        ("".into(), "".into())
                    };
                    writeln!(
                        md,
                        "| {} | {} | {} | {} | {:.1} | {:.2} |",
                        r.axis_name, d_col, c_col, label, freq, amp
                    )?;
                }
            }
        }
        writeln!(md)?;
    }

    // --- Optimal P Estimation ---
    let has_optimal_p = report.optimal_p.iter().any(|o| o.is_some())
        || report.optimal_p_skip.iter().any(|s| s.is_some());
    if has_optimal_p {
        writeln!(md, "## Optimal P Estimation")?;
        writeln!(md)?;
        for (axis_index, opt) in report.optimal_p.iter().enumerate() {
            if opt.is_none() {
                if let Some(reason) = &report.optimal_p_skip[axis_index] {
                    writeln!(md, "### {}", AXIS_NAMES[axis_index])?;
                    writeln!(md)?;
                    writeln!(md, "- {}", reason)?;
                    writeln!(md)?;
                }
            }
            if let Some(analysis) = opt {
                writeln!(md, "### {}", AXIS_NAMES[axis_index])?;
                writeln!(md)?;
                let d_range = report.d_ranges[axis_index];
                let rec_text = match &analysis.recommendation {
                    rec @ PRecommendation::Optimal { reasoning, .. } => format!(
                        "{} — {}",
                        rec.optimal_short_text().unwrap_or_default(),
                        reasoning
                    ),
                    PRecommendation::Increase {
                        conservative_p,
                        reasoning,
                    } => format!(
                        "Increase P to {} ({:+}){} — {}",
                        conservative_p,
                        *conservative_p as i64 - analysis.current_p as i64,
                        analysis.d_target_suffix(*conservative_p, d_range),
                        reasoning
                    ),
                    PRecommendation::Decrease {
                        recommended_p,
                        reasoning,
                    } => format!(
                        "Decrease P to {} ({:+}){} — {}",
                        recommended_p,
                        *recommended_p as i64 - analysis.current_p as i64,
                        analysis.d_target_suffix(*recommended_p, d_range),
                        reasoning
                    ),
                    PRecommendation::Investigate { issue } => format!("Investigate — {}", issue),
                };
                // Order: recommendation first, then the evidence behind it.
                writeln!(md, "- **Recommendation:** {}", rec_text)?;
                writeln!(md, "- **Current P:** {}", analysis.current_p)?;
                match (report.d_ranges[axis_index], analysis.current_d) {
                    (Some((min, max)), _) => {
                        writeln!(md, "- **Current D:** D-Min {}, D-Max {}", min, max)?
                    }
                    (None, Some(d)) => writeln!(md, "- **Current D:** {}", d)?,
                    (None, None) => {}
                }
                writeln!(
                    md,
                    "- **Td measured (median):** {:.1} ms",
                    analysis.td_stats.median_ms
                )?;
                writeln!(
                    md,
                    "- **Td target:** {:.1} ms ± {:.1} ms",
                    analysis.td_target_ms, analysis.td_tolerance_ms
                )?;
                if let Some(clip) = analysis.flight_clip_text() {
                    writeln!(md, "- **Flight clip:** {}", clip)?;
                }
                writeln!(
                    md,
                    "- **Td deviation:** {:.1}% ({})",
                    analysis.td_deviation_percent,
                    analysis.td_deviation.name()
                )?;
                writeln!(md, "- **Noise level:** {}", analysis.noise_level.name())?;
                writeln!(md, "- **Source:** {}", analysis.source_summary())?;
                writeln!(
                    md,
                    "- **{}:** {}",
                    analysis.td_stats.reliability_label(),
                    analysis.td_stats.reliability_details()
                )?;
                writeln!(md)?;
            }
        }
    }

    // --- Bode Analysis ---
    if !report.bode_results.is_empty() {
        writeln!(md, "## Bode Analysis")?;
        writeln!(md)?;
        writeln!(
            md,
            "> ⚠ Bode analysis is designed for controlled system-identification test flights."
        )?;
        writeln!(md)?;
        writeln!(
            md,
            "| Axis | Phase Margin | Gain Margin | Gain Crossover | Bandwidth | Confidence |"
        )?;
        writeln!(
            md,
            "|------|-------------|-------------|----------------|-----------|------------|"
        )?;
        for r in &report.bode_results {
            let m = &r.margins;
            let pm = m
                .phase_margin_deg
                .map_or("N/A".into(), |v| format!("{:.1}°", v));
            let gm = m
                .gain_margin_db
                .map_or("N/A".into(), |v| format!("{:.1} dB", v));
            let gc = m
                .gain_crossover_hz
                .map_or("N/A".into(), |v| format!("{:.2} Hz", v));
            let bw = m
                .bandwidth_hz
                .map_or("N/A".into(), |v| format!("{:.2} Hz", v));
            let conf = match m.confidence {
                Confidence::High => "High",
                Confidence::Medium => "Medium",
                Confidence::Low => "Low",
            };
            writeln!(
                md,
                "| {} | {} | {} | {} | {} | {} |",
                r.axis_name, pm, gm, gc, bw, conf
            )?;
        }
        writeln!(md)?;
    }

    // --- Motor Oscillation ---
    if !report.motor_results.is_empty() {
        writeln!(md, "## Motor Oscillation")?;
        writeln!(md)?;
        writeln!(
            md,
            "Analysis range: {:.0}–{:.0} Hz",
            MOTOR_OSCILLATION_FREQ_MIN_HZ, MOTOR_OSCILLATION_FREQ_MAX_HZ
        )?;
        writeln!(
            md,
            "Peak/Avg/Event Time reflect the single worst {:.0}ms window found across the whole flight, not a whole-log average — a brief oscillation burst is diluted away by averaging over a multi-minute flight.",
            MOTOR_OSCILLATION_WINDOW_S * MOTOR_OSCILLATION_SECONDS_TO_MS
        )?;
        writeln!(md)?;
        writeln!(
            md,
            "| Motor | Max Amplitude | Oscillation | Peak in Window | Avg in Window | Event Time (s) |"
        )?;
        writeln!(
            md,
            "|-------|--------------|-------------|-----------------|----------------|-----------------|"
        )?;
        for r in &report.motor_results {
            let max_amp = r
                .max_amplitude
                .map_or("N/A".into(), |v| format!("{:.2}", v));
            let osc = if r.oscillation_detected {
                "⚠ Detected"
            } else {
                "None"
            };
            let peak = r
                .peak_in_range
                .map_or("N/A".into(), |v| format!("{:.2}", v));
            let avg = r.avg_in_range.map_or("N/A".into(), |v| format!("{:.2}", v));
            let event_time = r.event_time_s.map_or("N/A".into(), |v| format!("{:.2}", v));
            writeln!(
                md,
                "| {} | {} | {} | {} | {} | {} |",
                r.motor_idx, max_amp, osc, peak, avg, event_time
            )?;
        }
        writeln!(md)?;
    }

    // --- Motor Desync Detection ---
    if !report.motor_desync_results.is_empty() {
        writeln!(md, "## Motor Desync Detection")?;
        writeln!(md)?;
        let fallback_active = report.motor_desync_results.iter().any(|r| r.fallback_used);
        if fallback_active {
            writeln!(
                md,
                "No eRPM telemetry in this log. Flags a motor near its own command ceiling when the aircraft's rotation diverges sharply from setpoint, self-relative to this flight's own distributions. **Fallback** cannot confirm actual motor RPM response — it is the least confident tier, and can miss real desyncs or flag legitimate hard maneuvers. Cross-check every flagged time against gyro/setpoint traces and video."
            )?;
        } else {
            writeln!(
                md,
                "Flags a motor commanded high whose eRPM fails to respond, self-relative to that motor's own behavior this flight. **De Facto** uses a rich same-motor baseline. **Possible** is a looser check for flights with too little high-command history. This is a heuristic, not a confirmed diagnosis. Cross-check flagged times against gyro/accelerometer disturbance."
            )?;
        }
        writeln!(md)?;
        writeln!(
            md,
            "| Motor | eRPM Telemetry | De Facto | Possible | Fallback | De Facto Times (s) | Possible Times (s) | Fallback Times (s) |"
        )?;
        writeln!(
            md,
            "|-------|-----------------|----------|----------|----------|----------------------|-----------------------|----------------------|"
        )?;
        let mut any_de_facto = false;
        let mut any_possible = false;
        let mut any_fallback = false;
        for r in &report.motor_desync_results {
            if !r.erpm_signal_available && !r.fallback_used {
                writeln!(
                    md,
                    "| {} | Insufficient signal | N/A | N/A | N/A | N/A | N/A | N/A |",
                    r.motor_idx
                )?;
                continue;
            }
            let fmt_times = |confidence: DesyncConfidence| -> String {
                let matching: Vec<f64> = r
                    .events
                    .iter()
                    .filter(|e| e.confidence == confidence)
                    .map(|e| e.time_s)
                    .collect();
                if matching.is_empty() {
                    return "N/A".to_string();
                }
                let times: Vec<String> = matching
                    .iter()
                    .take(MOTOR_DESYNC_REPORT_MAX_TIMES)
                    .map(|t| format!("{t:.2}"))
                    .collect();
                if matching.len() > MOTOR_DESYNC_REPORT_MAX_TIMES {
                    format!("{}, ...", times.join(", "))
                } else {
                    times.join(", ")
                }
            };
            let count = |confidence: DesyncConfidence| -> usize {
                r.events
                    .iter()
                    .filter(|e| e.confidence == confidence)
                    .count()
            };
            let de_facto_count = count(DesyncConfidence::DeFacto);
            let possible_count = count(DesyncConfidence::Possible);
            let fallback_count = count(DesyncConfidence::Fallback);
            any_de_facto |= de_facto_count > 0;
            any_possible |= possible_count > 0;
            any_fallback |= fallback_count > 0;
            let telemetry = if r.fallback_used {
                "Fallback (no eRPM)"
            } else {
                "Available"
            };
            writeln!(
                md,
                "| {} | {} | {} | {} | {} | {} | {} | {} |",
                r.motor_idx,
                telemetry,
                de_facto_count,
                possible_count,
                fallback_count,
                fmt_times(DesyncConfidence::DeFacto),
                fmt_times(DesyncConfidence::Possible),
                fmt_times(DesyncConfidence::Fallback)
            )?;
        }
        writeln!(md)?;
        if any_de_facto || any_possible || any_fallback {
            writeln!(
                md,
                "**⚠ Recommendation:** Review the flagged timestamps against gyro traces and audio/video for the same moments. Inspect the affected motor, ESC, and prop for damage or a loose connection."
            )?;
            writeln!(md)?;
        }
        let overlaps =
            fallback_oscillation_overlaps(&report.motor_desync_results, &report.motor_results);
        if !overlaps.is_empty() {
            for (motor_idx, count) in &overlaps {
                writeln!(
                    md,
                    "**⚠ Note:** Motor {motor_idx}'s Fallback event(s) ({count} overlapping event(s)) coincides with a Motor Oscillation detection — may be chronic tune/mechanical resonance rather than a desync. See Fallback column above for exact timestamps."
                )?;
            }
            writeln!(md)?;
        }
    }

    // --- Stick Input Smoothness (RC Command step detection) ---
    let has_rc_command_data = report
        .rc_command_steps
        .iter()
        .any(|r| r.median_plateau_ms.is_some());
    if has_rc_command_data {
        writeln!(md, "## Stick Input Smoothness")?;
        writeln!(md)?;
        writeln!(
            md,
            "| Axis | Step Count | Median Plateau (ms) | Assessment |"
        )?;
        writeln!(
            md,
            "|------|-----------|---------------------|------------|"
        )?;
        let mut any_blocky = false;
        for r in &report.rc_command_steps {
            let plateau = r
                .median_plateau_ms
                .map_or("N/A".into(), |v| format!("{:.1}", v));
            let assessment = if r.median_plateau_ms.is_none() {
                "N/A"
            } else if r.is_blocky {
                any_blocky = true;
                "⚠ Blocky"
            } else {
                "Smooth"
            };
            writeln!(
                md,
                "| {} | {} | {} | {} |",
                r.axis_name, r.step_count, plateau, assessment
            )?;
        }
        writeln!(md)?;
        if any_blocky {
            writeln!(
                md,
                "**⚠ Recommendation:** Discrete steps detected in RC Command on one or more axes. Raw, unsmoothed stick input can appear as jitter in the Setpoint response. Review the aircraft's `rc_smoothing_*` settings, or confirm the RX link update rate."
            )?;
            writeln!(md)?;
        }
    }

    // --- Stick Position & Rate Analysis ---
    let has_stick_distribution_data = report
        .stick_distribution_results
        .iter()
        .any(|r| r.peak_stick.is_some());
    if has_stick_distribution_data {
        writeln!(md, "## Stick Position & Rate Analysis")?;
        writeln!(md)?;
        writeln!(
            md,
            "| Axis | Peak Stick | Center | Mid | High | Saturation | Saturation (s) | Saturation Events | Center Reversal Rate (Hz) |"
        )?;
        writeln!(
            md,
            "|------|------------|--------|-----|------|------------|-----------------|--------------------|---------------------------|"
        )?;
        for r in &report.stick_distribution_results {
            if r.peak_stick.is_none() {
                writeln!(
                    md,
                    "| {} | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A |",
                    r.axis_name
                )?;
                continue;
            }
            let peak_stick_pct =
                r.peak_stick.unwrap_or(0.0) / STICK_DIST_FULL_STICK_RC_COMMAND * RATIO_TO_PERCENT;
            let reversal_rate = r
                .center_reversal_rate_hz
                .map_or("N/A".into(), |v| format!("{:.2}", v));
            writeln!(
                md,
                "| {} | {} | {} | {} | {} | {} | {:.1} | {} | {} |",
                r.axis_name,
                fmt_zone_pct(peak_stick_pct),
                fmt_zone_pct(r.center_pct),
                fmt_zone_pct(r.mid_pct),
                fmt_zone_pct(r.high_pct),
                fmt_zone_pct(r.saturation_pct),
                r.saturation_time_s,
                r.saturation_event_count,
                reversal_rate
            )?;
        }
        writeln!(md)?;
        writeln!(
            md,
            "Peak Stick is the highest |rc_command| this flight reached, as a % of true full-stick range (both Betaflight and EmuFlight clamp rcCommand to +/-500 before the rate curve). Center/Mid/High are % of flight time spent below 15%, 15-75%, and above 75% of that same true full-stick range — not this axis's own peak; these three always sum to 100%. Saturation is the % of flight time above 95% — a subset of High, not additional to it. Saturation (s) and Saturation Events report that same threshold in seconds and as a count of separate excursions, not cumulative time. A flight whose Peak Stick never reaches 95% cannot register a Saturation Event."
        )?;
        writeln!(md)?;

        writeln!(
            md,
            "| Axis | P95 Setpoint (deg/s) | P95 Gyro Achieved (deg/s) | Configured Max Rate (deg/s) | Rate Headroom (P95) |"
        )?;
        writeln!(
            md,
            "|------|----------------------|----------------------------|------------------------------|----------------------|"
        )?;
        for r in &report.stick_distribution_results {
            let p95_setpoint = r.p95_setpoint.map_or("N/A".into(), |v| format!("{:.0}", v));
            let p95_gyro = r.p95_gyro.map_or("N/A".into(), |v| format!("{:.0}", v));
            let configured_max_rate = r
                .configured_max_rate
                .map_or("N/A".into(), |v| format!("{:.0}", v));
            let rate_headroom = r.rate_headroom_pct.map_or("N/A".into(), fmt_zone_pct);
            writeln!(
                md,
                "| {} | {} | {} | {} | {} |",
                r.axis_name, p95_setpoint, p95_gyro, configured_max_rate, rate_headroom
            )?;
        }
        writeln!(
            md,
            "\nP95 Setpoint/P95 Gyro Achieved use the 95th percentile, not the flight's raw maximum — a single crash/tumble sample can put raw max gyro rate an order of magnitude above the rest of the flight. Configured Max Rate is the setpoint at full stick deflection, computed from this log's own rc_rates/rc_expo/rates/rates_type/rate_limits headers — N/A when the header set is incomplete. Rate Headroom is the unused portion of Configured Max Rate: 100% minus (P95 Setpoint as a % of Configured Max Rate). For tracking quality (how closely gyro follows setpoint), see the Step Response Analysis section above — not duplicated here."
        )?;
        writeln!(md)?;
    }

    // --- Gyro Vibration Percentile Analysis ---
    let has_vibration_data = report
        .vibration_percentile_results
        .iter()
        .any(|r| r.filtered.is_some() || r.unfiltered.is_some());
    if has_vibration_data {
        writeln!(md, "## Gyro Vibration Percentile Analysis")?;
        writeln!(md)?;
        let header_cols: Vec<String> = VIBRATION_PERCENTILES_PCT
            .iter()
            .map(|p| format!("P{p} (deg/s)"))
            .collect();
        writeln!(
            md,
            "| Axis | Signal | Samples | {} |",
            header_cols.join(" | ")
        )?;
        writeln!(
            md,
            "|------|--------|---------|{}",
            "------|".repeat(VIBRATION_PERCENTILES_PCT.len())
        )?;
        for r in &report.vibration_percentile_results {
            for (signal, series) in [("Filtered", &r.filtered), ("Unfiltered", &r.unfiltered)] {
                if let Some(s) = series {
                    let cells: Vec<String> = s
                        .percentiles
                        .iter()
                        .map(|v| format_percentile_value(*v))
                        .collect();
                    writeln!(
                        md,
                        "| {} | {} | {} | {} |",
                        r.axis_name,
                        signal,
                        s.count,
                        cells.join(" | ")
                    )?;
                }
            }
        }
        writeln!(md)?;
        writeln!(
            md,
            "Percentiles of |gyro| over all finite samples of the flight, value at sorted index (n-1)*p/100. They describe the whole signal, including intentional maneuvers, not noise alone: compare Filtered with Unfiltered, and compare flights flown the same way. A P99 far above P95 means rare large excursions; P50 close to P99 means steady vibration."
        )?;
        writeln!(md)?;
    }

    // --- Generated Plots ---
    if !report.png_links.is_empty() {
        writeln!(md, "## Generated Plots")?;
        writeln!(md)?;
        for name in &report.png_links {
            writeln!(md, "- [{}]({})", name, name)?;
        }
        writeln!(md)?;
    }

    // --- Skipped Plots ---
    if !report.skipped_plots.is_empty() {
        writeln!(md, "## Skipped Plots")?;
        writeln!(md)?;
        writeln!(md, "No plottable data:")?;
        for name in &report.skipped_plots {
            writeln!(md, "- {name}")?;
        }
        writeln!(md)?;
    }

    fs::write(output_path, md)?;
    Ok(())
}

/// Formats a percentage with one decimal place, so a small nonzero value (e.g. 0.34%)
/// doesn't round away to "0%" under whole-number formatting.
fn fmt_zone_pct(pct: f64) -> String {
    format!("{:.1}%", pct)
}

fn write_rpm_filter_lines(
    md: &mut String,
    rpm: &RpmFilterConfig,
    frequencies: Option<&RpmMotorFrequencies>,
) -> Result<(), Box<dyn Error>> {
    writeln!(
        md,
        "- **RPM Filter:** {} harmonic{}, Q={:.0}, min {:.0} Hz, weights: [{}]",
        rpm.harmonics,
        if rpm.harmonics == 1 { "" } else { "s" },
        rpm.q_factor,
        rpm.min_hz,
        rpm.weights_percent_display()
    )?;
    for (label, text) in rpm_motor_frequency_lines(rpm, frequencies) {
        writeln!(md, "- **RPM Filter {label}:** {text}")?;
    }
    Ok(())
}

fn fmt_static_filter(f: &crate::data_analysis::filter_response::FilterConfig) -> String {
    match f.q_factor {
        Some(q) => format!(
            "{} @ {:.0} Hz (Q={:.2})",
            f.filter_type.name(),
            f.cutoff_hz,
            q
        ),
        None => format!("{} @ {:.0} Hz", f.filter_type.name(), f.cutoff_hz),
    }
}

fn fmt_filter_stage(
    lpf1: Option<&crate::data_analysis::filter_response::FilterConfig>,
    dyn_lpf: Option<&crate::data_analysis::filter_response::DynamicFilterConfig>,
) -> String {
    if let Some(f) = lpf1 {
        fmt_static_filter(f)
    } else if let Some(d) = dyn_lpf {
        format!(
            "Dyn {} {:.0}–{:.0} Hz",
            d.filter_type.name(),
            d.min_cutoff_hz,
            d.max_cutoff_hz
        )
    } else {
        "—".into()
    }
}

fn fmt_imuf(f: &crate::data_analysis::filter_response::ImufFilterConfig) -> String {
    if f.lowpass_cutoff_hz > 0.0 {
        let stages = match f.ptn_order {
            1 => "PT1",
            2 => "2×PT1",
            3 => "3×PT1",
            4 => "4×PT1",
            _ => "PT?",
        };
        let rev = f.revision.map_or("IMUF".into(), |r| format!("IMUF v{}", r));
        let w_part = f
            .pseudo_kalman_w
            .map_or(String::new(), |w| format!(", w={:.0}", w));
        format!(
            "{}: {} Combined={:.0} Hz per-stage={:.0} Hz (Q={:.1}{})",
            rev, stages, f.lowpass_cutoff_hz, f.effective_cutoff_hz, f.q_factor, w_part
        )
    } else {
        let w_part = f
            .pseudo_kalman_w
            .map_or(String::new(), |w| format!(", w={:.0}", w));
        format!("Pseudo-Kalman Q={:.1}{}", f.q_factor, w_part)
    }
}

fn fmt_dterm_rec(rec: &DTermRec) -> String {
    // A range end present means the axis has a valid D-Min/D-Max pair (see main.rs callers).
    match format_d_target(
        rec.d,
        rec.d_min,
        rec.d_max,
        rec.d_min.is_some() || rec.d_max.is_some(),
    ) {
        Some(d_text) => format!("P:D={:.2} ({})", rec.pd_ratio, d_text),
        None => format!("P:D={:.2}", rec.pd_ratio),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_analysis::calc_step_response::{displayed_response, find_peak_value};
    use ndarray::{Array1, Array2};

    fn empty_report(step_reports: Vec<StepAxisReport>) -> FlightReport {
        FlightReport {
            root_name: "peak_parity".to_string(),
            sample_rate: Some(1000.0),
            trim_window: None,
            header_metadata: Vec::new(),
            pd_ratios: Default::default(),
            pid_labels: Default::default(),
            d_ranges: Default::default(),
            step_reports,
            optimal_p: Default::default(),
            optimal_p_skip: Default::default(),
            gyro_analysis: None,
            dterm_results: Vec::new(),
            bode_results: Vec::new(),
            motor_results: Vec::new(),
            motor_desync_results: Vec::new(),
            rc_command_steps: Vec::new(),
            stick_distribution_results: Vec::new(),
            vibration_percentile_results: Vec::new(),
            png_links: Vec::new(),
            skipped_plots: Vec::new(),
            filter_config: None,
            dynamic_notch: None,
            rpm_filter: None,
            rpm_motor_frequencies: None,
            debug_fallback: false,
            debug_mode_name: None,
        }
    }

    // Spec: the report prints the peak of the shared displayed response to three decimals.
    // Steady state 4.0 with one 4.8 sample gives a peak of exactly 1.200.
    #[test]
    fn report_carries_the_shared_displayed_response_peak() {
        let len = 500;
        let stacked = Array2::from_shape_fn((2, len), |(_, i)| if i == 100 { 4.8f32 } else { 4.0 });
        let mask = Array1::<f32>::ones(2);
        let resp = displayed_response(&stacked, &mask, len, 1000.0).expect("response");
        let peak = find_peak_value(&resp).expect("peak");

        let step = StepAxisReport {
            axis_name: "Roll",
            peak_value: peak,
            assessment: "Overshoot",
            current_pd_ratio: 1.2,
            conservative: None,
            moderate: None,
            aggressive: None,
            near_optimal_hint: None,
            setpoint_authority_name: None,
            setpoint_authority_mean: None,
            warnings: Vec::new(),
        };
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("report.md");
        generate_markdown_report(&empty_report(vec![step]), &path).expect("report");
        let text = fs::read_to_string(&path).expect("read");
        assert!(text.contains("- **Peak Value:** 1.200"), "{text}");
    }

    // Spec: a skipped axis appears in the report with the same reason text as the console and
    // legend, and no recommendation.
    #[test]
    fn report_carries_the_optimal_p_skip_reason() {
        let reason = "SKIPPED: insufficient throttle dynamics (2 events, need >=5)".to_string();
        let mut report = empty_report(Vec::new());
        report.optimal_p_skip[0] = Some(reason.clone());
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("report.md");
        generate_markdown_report(&report, &path).expect("report");
        let text = fs::read_to_string(&path).expect("read");
        assert!(text.contains("## Optimal P Estimation"), "{text}");
        assert!(text.contains(&format!("- {reason}")), "{text}");
        assert!(!text.contains("**Recommendation:**"), "{text}");
    }

    // Spec: the report carries the shared Optimal short text, then the long reasoning.
    #[test]
    fn report_carries_the_shared_optimal_short_text() {
        // Target 25 +- 3 ms, P 60, low noise; 18 ms is -28% against the target (Case 8).
        let analysis = OptimalPAnalysis::analyze(
            &[18.0; 40],
            60,
            Some(20),
            Some(0.01),
            Some(2.0),
            Some((25.0, 3.0)),
        )
        .expect("analysis");
        let short = analysis
            .recommendation
            .optimal_short_text()
            .expect("optimal");
        let mut report = empty_report(Vec::new());
        report.optimal_p[0] = Some(analysis);
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("report.md");
        generate_markdown_report(&report, &path).expect("report");
        let text = fs::read_to_string(&path).expect("read");
        assert!(text.contains(&short), "{text}");
        assert!(
            text.contains("more P headroom is possible but not quantified"),
            "{text}"
        );
    }

    // Spec: the report table carries the shared percentile formatter's values (1..=100 gives
    // P50 50, P90 90, P95 95, P99 99).
    #[test]
    fn report_carries_the_shared_vibration_percentiles() {
        use crate::data_analysis::vibration_percentile::build_series;
        let mut report = empty_report(Vec::new());
        report.vibration_percentile_results = vec![VibrationPercentileResult {
            axis_name: "Roll".to_string(),
            filtered: build_series((1..=100).map(f64::from)),
            unfiltered: None,
        }];
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("report.md");
        generate_markdown_report(&report, &path).expect("report");
        let text = fs::read_to_string(&path).expect("read");
        assert!(
            text.contains("## Gyro Vibration Percentile Analysis"),
            "{text}"
        );
        assert!(
            text.contains("| Roll | Filtered | 100 | 50.0 | 90.0 | 95.0 | 99.0 |"),
            "{text}"
        );
        assert!(!text.contains("| Roll | Unfiltered |"), "{text}");
    }
}
