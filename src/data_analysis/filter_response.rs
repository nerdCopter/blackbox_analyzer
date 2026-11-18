// src/data_analysis/filter_response.rs

use std::collections::HashMap;

use crate::axis_names::AXIS_NAMES;
use crate::constants::{
    PARITY_DIVISOR, RATIO_TO_PERCENT, RPM_FILTER_BAND_HIGH_PERCENTILE,
    RPM_FILTER_BAND_LOW_PERCENTILE, RPM_FILTER_CURVE_MIN_ATTENUATION,
    RPM_FILTER_CURVE_START_FLOOR_HZ, RPM_FILTER_CURVE_START_FRACTION,
    RPM_FILTER_DEFAULT_GYRO_RATE_HZ, RPM_FILTER_ERPM_PER_LSB, RPM_FILTER_MAX_HZ_RATE_FRACTION,
    RPM_FILTER_MICROSECONDS_PER_SECOND, RPM_FILTER_MIN_MOTOR_POLES, RPM_FILTER_POLES_PER_PAIR,
    RPM_FILTER_SECONDS_PER_MINUTE, RPM_HEATMAP_MIN_BIN_SAMPLES, RPM_HEATMAP_MIN_LINE_POINTS,
};
use crate::data_input::log_data::LogRowData;

/// Filter types supported by flight controllers
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilterType {
    PT1 = 0,
    Biquad = 1,
    PT2 = 2,
    PT3 = 3,
    PT4 = 4, // EmuFlight specific, rarely used but supported
}

impl FilterType {
    /// Convert numeric filter type to enum
    pub fn from_u32(value: u32) -> Option<Self> {
        match value {
            0 => Some(FilterType::PT1),
            1 => Some(FilterType::Biquad),
            2 => Some(FilterType::PT2),
            3 => Some(FilterType::PT3),
            4 => Some(FilterType::PT4),
            _ => None,
        }
    }

    /// Get filter type name for display
    pub fn name(&self) -> &'static str {
        match self {
            FilterType::PT1 => "PT1",
            FilterType::Biquad => "BIQUAD",
            FilterType::PT2 => "PT2",
            FilterType::PT3 => "PT3",
            FilterType::PT4 => "PT4",
        }
    }
}

/// Filter configuration for a single filter stage
#[derive(Debug, Clone)]
pub struct FilterConfig {
    pub filter_type: FilterType,
    pub cutoff_hz: f64,
    pub q_factor: Option<f64>, // Q-factor for BIQUAD filters (None defaults to 0.707 Butterworth)
    pub enabled: bool,
}

/// Filter configuration for dynamic filters (Betaflight)
#[derive(Debug, Clone)]
pub struct DynamicFilterConfig {
    pub filter_type: FilterType,
    pub min_cutoff_hz: f64,
    pub max_cutoff_hz: f64,
    pub expo: u32,
    pub enabled: bool,
}

/// IMUF filter configuration with Butterworth correction (Betaflight/EmuFlight)
#[derive(Debug, Clone)]
pub struct ImufFilterConfig {
    // Header values (as configured)
    pub lowpass_cutoff_hz: f64, // Configured combined cutoff from header (effective -3dB point)
    pub ptn_order: u32,         // Filter order (1-4 -> PT1, PT2, PT3, PT4)
    pub q_factor: f64,          // Q-factor (scaled by 1000 in headers)
    pub revision: Option<u32>,  // IMUF revision from header (e.g., 256, 257)
    pub pseudo_kalman_w: Option<f64>, // Pseudo-Kalman filter window size (IMUF_w parameter)

    // Calculated values (Butterworth correction for cascaded stages)
    pub effective_cutoff_hz: f64, // Per-stage PT1 cutoff (scaled up to achieve configured combined response)

    pub enabled: bool,
}

#[derive(Debug, Clone, Default)]
pub struct AxisFilterConfig {
    pub lpf1: Option<FilterConfig>,
    pub lpf2: Option<FilterConfig>,
    pub dynamic_lpf1: Option<DynamicFilterConfig>,
    pub imuf: Option<ImufFilterConfig>,
}

/// Filter configuration for all gyro and dterm filters
#[derive(Debug, Clone, Default)]
pub struct AllFilterConfigs {
    pub gyro: [AxisFilterConfig; 3],  // Roll, Pitch, Yaw
    pub dterm: [AxisFilterConfig; 3], // Roll, Pitch, Yaw
}

/// Type alias for filter curve data: (label, curve_points, cutoff_hz)
pub type FilterCurveData = (String, Vec<(f64, f64)>, f64);

/// Type alias for RPM filter curve data: (harmonic_num, label, curve_points, center_hz)
pub type RpmFilterCurveData = (u32, String, Vec<(f64, f64)>, f64);

/// Helper function to add PTn filter curves with Butterworth correction
/// Generates both the user-configured curve and the per-stage implementation curve
#[allow(clippy::too_many_arguments)]
fn add_ptn_filter_curves(
    filter_curves: &mut Vec<FilterCurveData>,
    filter_type: FilterType,
    cutoff_hz: f64,
    q_factor: Option<f64>,
    label_prefix: &str,
    max_frequency_hz: f64,
    num_points: usize,
    show_butterworth: bool,
) {
    // Generate curve for user-configured cutoff (e.g., PT2 @ 90Hz)
    let main_filter = FilterConfig {
        filter_type,
        cutoff_hz,
        q_factor,
        enabled: true,
    };
    let main_curve = generate_single_filter_curve(&main_filter, max_frequency_hz, num_points);
    let main_label = if let Some(q) = q_factor {
        format!(
            "{} ({} @ {:.0}Hz, Q={:.2})",
            label_prefix,
            filter_type.name(),
            cutoff_hz,
            q
        )
    } else {
        format!(
            "{} ({} @ {:.0}Hz)",
            label_prefix,
            filter_type.name(),
            cutoff_hz
        )
    };
    filter_curves.push((main_label, main_curve, cutoff_hz));

    // Generate per-stage curve for PT2/PT3/PT4 (Butterworth correction) only if --butterworth flag is set
    if !show_butterworth {
        return;
    }

    let ptn_order = match filter_type {
        FilterType::PT2 => 2,
        FilterType::PT3 => 3,
        FilterType::PT4 => 4,
        _ => 0, // PT1 and BIQUAD don't need per-stage curves
    };

    if ptn_order > 1 {
        let per_stage_cutoff = calculate_ptn_per_stage_cutoff(cutoff_hz, ptn_order);

        // Only show per-stage if it's meaningfully different (> 1 Hz)
        if (per_stage_cutoff - cutoff_hz).abs() > 1.0 {
            let stage_count_text = match ptn_order {
                2 => "Two PT1",
                3 => "Three PT1",
                4 => "Four PT1",
                _ => "PT1",
            };

            let stage_filter = FilterConfig {
                filter_type,
                cutoff_hz: per_stage_cutoff,
                q_factor: None, // PT1 stages don't have Q-factor
                enabled: true,
            };
            let stage_curve =
                generate_single_filter_curve(&stage_filter, max_frequency_hz, num_points);
            let stage_label = format!(
                "≈ {} ({} @ {:.0}Hz per-stage)",
                label_prefix, stage_count_text, per_stage_cutoff
            );
            filter_curves.push((stage_label, stage_curve, per_stage_cutoff));
        }
    }
}

/// Calculate frequency response magnitude for PT1 filter
/// H(s) = 1 / (1 + s/ωc) where ωc = 2π * cutoff_hz
pub fn pt1_response(frequency_hz: f64, cutoff_hz: f64) -> f64 {
    if cutoff_hz <= 0.0 {
        return 1.0; // No filtering
    }
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let omega_c = 2.0 * std::f64::consts::PI * cutoff_hz;
    1.0 / (1.0 + (omega / omega_c).powi(2)).sqrt()
}

/// Calculate frequency response magnitude for PT2 filter (2nd order Butterworth)
/// H(s) = 1 / (1 + √2·s/ωc + (s/ωc)²)
pub fn pt2_response(frequency_hz: f64, cutoff_hz: f64) -> f64 {
    if cutoff_hz <= 0.0 {
        return 1.0; // No filtering
    }
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let omega_c = 2.0 * std::f64::consts::PI * cutoff_hz;
    let s_norm = omega / omega_c;
    // Butterworth 2nd order magnitude: 1 / sqrt(1 + (ω/ωc)^4)
    1.0 / (1.0 + s_norm.powi(4)).sqrt()
}

/// Calculate frequency response magnitude for PT3 filter (3rd order Butterworth)  
/// H(s) = 1 / (1 + s + s² + s³) where s = jω/ωc
pub fn pt3_response(frequency_hz: f64, cutoff_hz: f64) -> f64 {
    if cutoff_hz <= 0.0 {
        return 1.0; // No filtering
    }
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let omega_c = 2.0 * std::f64::consts::PI * cutoff_hz;
    let s_norm = omega / omega_c;
    // 3rd order Butterworth: 1 / sqrt((1 + s²)² + s²)
    // Approximated as: 1 / sqrt(1 + s⁶) for simplicity while maintaining -3dB at cutoff
    1.0 / (1.0 + s_norm.powi(6)).sqrt()
}

/// Calculate frequency response magnitude for PT4 filter (4th order Butterworth)
/// H(s) = 1 / (1 + √2·s + s² + √2·s³ + s⁴)
/// EmuFlight specific, rarely used but supported for completeness
pub fn pt4_response(frequency_hz: f64, cutoff_hz: f64) -> f64 {
    if cutoff_hz <= 0.0 {
        return 1.0; // No filtering
    }
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let omega_c = 2.0 * std::f64::consts::PI * cutoff_hz;
    let s_norm = omega / omega_c;
    // 4th order Butterworth: 1 / sqrt(1 + s⁸) approximation maintaining -3dB at cutoff
    1.0 / (1.0 + s_norm.powi(8)).sqrt()
}

/// Calculate frequency response magnitude for BIQUAD filter with variable Q-factor
/// H(s) = ω₀² / (s² + (ω₀/Q)·s + ω₀²)
/// Frequency response magnitude: |H(jω)| = 1 / sqrt((1 - (ω/ω₀)²)² + (ω/(Q·ω₀))²)
///
/// Parameters:
/// - frequency_hz: Frequency to evaluate at
/// - cutoff_hz: -3dB cutoff frequency (ω₀ = 2π·fc)
/// - q_factor: Quality factor (default 0.707 for Butterworth, can range 0.5-10.0)
pub fn biquad_response_with_q(frequency_hz: f64, cutoff_hz: f64, q_factor: f64) -> f64 {
    if frequency_hz <= 0.0 || cutoff_hz <= 0.0 || q_factor <= 0.0 {
        return 1.0; // No filtering
    }

    // Clamp Q-factor to documented safe range (0.5-10.0) to prevent numerical instabilities
    let q_factor = q_factor.clamp(0.5, 10.0);

    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let omega_0 = 2.0 * std::f64::consts::PI * cutoff_hz;

    // Normalized frequency ratio
    let ratio = omega / omega_0;
    let ratio_sq = ratio * ratio;

    // |H(jω)| = 1 / sqrt((1 - r²)² + (r/Q)²)
    // where r = ω/ω₀
    let numerator = (1.0 - ratio_sq).powi(2) + (ratio / q_factor).powi(2);
    1.0 / numerator.sqrt()
}

/// Calculate frequency response magnitude for BIQUAD filter (backward compatible)
/// Uses default Butterworth Q-factor of 0.707 (1/sqrt(2))
#[allow(dead_code)]
pub fn biquad_response(frequency_hz: f64, cutoff_hz: f64) -> f64 {
    if cutoff_hz <= 0.0 {
        return 1.0; // No filtering
    }
    // Default to Butterworth response (Q = 1/sqrt(2) ≈ 0.707)
    let butterworth_q = std::f64::consts::FRAC_1_SQRT_2;
    biquad_response_with_q(frequency_hz, cutoff_hz, butterworth_q)
}

/// Generate individual filter response curves (separate curve for each filter)
/// Returns Vec<FilterCurveData> for multiple curves with cutoff markers
/// Ensures LPF1 (static or dynamic) appears before LPF2 in legend order
pub fn generate_individual_filter_curves(
    axis_config: &AxisFilterConfig,
    max_frequency_hz: f64,
    num_points: usize,
    show_butterworth: bool,
) -> Vec<FilterCurveData> {
    let mut filter_curves = Vec::new();

    // Generate curve for LPF1 (static or dynamic - whichever is configured)
    if let Some(ref dyn_lpf1) = axis_config.dynamic_lpf1 {
        // Dynamic LPF1 takes precedence
        if dyn_lpf1.enabled && dyn_lpf1.min_cutoff_hz > 0.0 {
            let label_prefix = if dyn_lpf1.max_cutoff_hz > dyn_lpf1.min_cutoff_hz {
                format!(
                    "Dyn LPF1 {:.0}-{:.0}Hz",
                    dyn_lpf1.min_cutoff_hz, dyn_lpf1.max_cutoff_hz
                )
            } else {
                "Dyn LPF1".to_string()
            };

            // Use helper to add both main and per-stage curves for PTn filters
            add_ptn_filter_curves(
                &mut filter_curves,
                dyn_lpf1.filter_type,
                dyn_lpf1.min_cutoff_hz,
                None, // Dynamic filters don't have Q-factor
                &label_prefix,
                max_frequency_hz,
                num_points,
                show_butterworth,
            );
        }
    } else if let Some(ref lpf1) = axis_config.lpf1 {
        // Static LPF1 fallback
        if lpf1.enabled && lpf1.cutoff_hz > 0.0 {
            // Use helper to add both main and per-stage curves for PTn filters
            add_ptn_filter_curves(
                &mut filter_curves,
                lpf1.filter_type,
                lpf1.cutoff_hz,
                lpf1.q_factor,
                "LPF1",
                max_frequency_hz,
                num_points,
                show_butterworth,
            );
        }
    }

    // Generate curve for LPF2 if enabled (always after LPF1)
    if let Some(ref lpf2) = axis_config.lpf2 {
        if lpf2.enabled && lpf2.cutoff_hz > 0.0 {
            // Use helper to add both main and per-stage curves for PTn filters
            add_ptn_filter_curves(
                &mut filter_curves,
                lpf2.filter_type,
                lpf2.cutoff_hz,
                lpf2.q_factor,
                "LPF2",
                max_frequency_hz,
                num_points,
                show_butterworth,
            );
        }
    }

    // Generate curves for IMUF filter if enabled - show both header and corrected values
    if let Some(ref imuf) = axis_config.imuf {
        if imuf.enabled && imuf.lowpass_cutoff_hz > 0.0 {
            let filter_type = match imuf.ptn_order {
                1 => FilterType::PT1,
                2 => FilterType::PT2,
                3 => FilterType::PT3,
                4 => FilterType::PT4,
                _ => FilterType::PT2, // Default fallback
            };

            // Helper to get stage count text for per-stage labels
            let stage_count_text = match imuf.ptn_order {
                1 => "One PT1",
                2 => "Two PT1",
                3 => "Three PT1",
                4 => "Four PT1",
                _ => "Two PT1",
            };

            // Build version string
            let version_str = if let Some(rev) = imuf.revision {
                format!("IMUF v{}", rev)
            } else {
                "IMUF".to_string()
            };

            // Generate curve for header-configured cutoff (user sees PT2 @ 90Hz)
            let header_filter = FilterConfig {
                filter_type,
                cutoff_hz: imuf.lowpass_cutoff_hz,
                q_factor: if imuf.q_factor > 0.0 {
                    Some(imuf.q_factor)
                } else {
                    None
                },
                enabled: true,
            };
            let header_curve =
                generate_single_filter_curve(&header_filter, max_frequency_hz, num_points);
            let header_label = format!(
                "{} ({} @ {:.0}Hz)",
                version_str,
                filter_type.name(),
                imuf.lowpass_cutoff_hz
            );
            filter_curves.push((header_label, header_curve, imuf.lowpass_cutoff_hz));

            // Generate curve for per-stage PT1 cutoff (Butterworth correction - shows Two PT1 @ 140Hz) only if --butterworth flag is set
            if show_butterworth && (imuf.effective_cutoff_hz - imuf.lowpass_cutoff_hz).abs() > 1.0 {
                let stage_filter = FilterConfig {
                    filter_type,
                    cutoff_hz: imuf.effective_cutoff_hz,
                    q_factor: None, // PT1 stages don't have Q-factor
                    enabled: true,
                };
                let stage_curve =
                    generate_single_filter_curve(&stage_filter, max_frequency_hz, num_points);
                let stage_label = format!(
                    "≈ {} ({} @ {:.0}Hz per-stage)",
                    version_str, stage_count_text, imuf.effective_cutoff_hz
                );
                filter_curves.push((stage_label, stage_curve, imuf.effective_cutoff_hz));
            }
        }
    }

    filter_curves
}

/// Generate response curve for a single filter with full frequency range
fn generate_single_filter_curve(
    filter: &FilterConfig,
    max_frequency_hz: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    let mut curve_points = Vec::with_capacity(num_points);

    if num_points < 2 {
        return curve_points;
    }

    // Standard practice: start at 10% of cutoff frequency to show complete response
    // This captures the flat passband, S-shaped transition, and roll-off regions
    let start_freq = filter.cutoff_hz * 0.1; // 10% of cutoff is standard

    if start_freq >= max_frequency_hz {
        return curve_points; // Invalid range
    }

    // Use logarithmic spacing for smooth curves over wide frequency ranges
    let log_start = start_freq.ln();
    let log_end = max_frequency_hz.ln();
    let log_step = (log_end - log_start) / (num_points - 1) as f64;

    // Generate the complete mathematically correct filter response curve
    for i in 0..num_points {
        let log_frequency = log_start + (i as f64 * log_step);
        let frequency = log_frequency.exp();
        let magnitude = match filter.filter_type {
            FilterType::PT1 => pt1_response(frequency, filter.cutoff_hz),
            FilterType::PT2 => pt2_response(frequency, filter.cutoff_hz),
            FilterType::PT3 => pt3_response(frequency, filter.cutoff_hz),
            FilterType::PT4 => pt4_response(frequency, filter.cutoff_hz),
            FilterType::Biquad => {
                // Use Q-factor if provided, otherwise default to Butterworth (1/sqrt(2) ≈ 0.707)
                let q = filter.q_factor.unwrap_or(std::f64::consts::FRAC_1_SQRT_2);
                biquad_response_with_q(frequency, filter.cutoff_hz, q)
            }
        };
        curve_points.push((frequency, magnitude));
    }

    curve_points
}

/// Check if gyro dynamic LPF filtering is enabled
/// Returns (has_dynamic, min_cutoff, max_cutoff)
/// Note: Betaflight (only firmware with dynamic LPF) uses same settings for all axes,
/// so we only need to check the first axis (Roll)
pub fn check_gyro_dynamic_lpf_usage(config: &AllFilterConfigs) -> (bool, f64, f64) {
    if let Some(ref dyn_lpf) = config.gyro[0].dynamic_lpf1 {
        if dyn_lpf.enabled
            && dyn_lpf.min_cutoff_hz > 0.0
            && dyn_lpf.max_cutoff_hz > dyn_lpf.min_cutoff_hz
        {
            return (true, dyn_lpf.min_cutoff_hz, dyn_lpf.max_cutoff_hz);
        }
    }
    (false, 0.0, 0.0)
}

/// Extract gyro rate from header metadata for proper Nyquist calculation
/// Filters operate at gyro rate, not logging rate
pub fn extract_gyro_rate(header_metadata: Option<&[(String, String)]>) -> Option<f64> {
    if let Some(metadata) = header_metadata {
        // Look for explicit gyro rate in various possible header formats
        for (key, value) in metadata {
            let key_l = key.to_ascii_lowercase();
            if key_l.contains("gyrosampleratehz")
                || key_l.contains("gyro_sample_hz")
                || key_l.contains("gyro_rate_hz")
                || key_l.contains("gyro_rate")
                || key_l.contains("gyrorate")
            {
                // Try to parse the gyro rate
                if let Ok(rate) = value.parse::<f64>() {
                    if rate > 1000.0 && rate <= 32000.0 {
                        // Reasonable gyro rate range
                        return Some(rate);
                    }
                }
            }
        }

        // Look for gyro sync denominator to calculate rate
        let mut base_rate = 8000.0; // Default PID loop rate
        let mut sync_denom = 1.0;

        for (key, value) in metadata {
            let key_l = key.to_ascii_lowercase();
            if key_l.contains("gyro_sync_denom") || key_l.contains("gyrosyncdenom") {
                if let Ok(denom) = value.parse::<f64>() {
                    if denom > 0.0 && denom <= 32.0 {
                        sync_denom = denom;
                    }
                }
            }
            // Some logs have explicit PID loop rate
            if key_l.contains("looptime") || key_l.contains("pid_process_denom") {
                if let Ok(val) = value.parse::<f64>() {
                    if val > 0.0 && val < 1000.0 {
                        // looptime in microseconds
                        base_rate = 1000000.0 / val;
                    } else if val > 1000.0 && val <= 32000.0 {
                        // direct frequency
                        base_rate = val;
                    }
                }
            }
        }

        let calculated_rate = base_rate / sync_denom;
        if calculated_rate > 1000.0 && calculated_rate <= 32000.0 {
            return Some(calculated_rate);
        }

        // Fallback: assume 8kHz for modern firmware
        for (key, value) in metadata {
            let key_l = key.to_ascii_lowercase();
            if (key_l.contains("firmwaretype") || key_l.contains("firmware"))
                && (value.contains("Betaflight")
                    || value.contains("EmuFlight")
                    || value.contains("INAV"))
            {
                return Some(8000.0); // 8kHz default for modern firmware
            }
        }
    }

    None // Will use default in calling code
}

/// Parse filter configurations from header metadata
pub fn parse_emuflight_filters(headers: &[(String, String)]) -> AllFilterConfigs {
    let header_map: HashMap<String, String> = headers
        .iter()
        .map(|(k, v)| (k.trim().to_lowercase(), v.trim().to_string()))
        .collect();

    let mut config = AllFilterConfigs::default();

    // Parse D-term filters (per-axis)
    for (axis_idx, axis_name) in ["roll", "pitch", "yaw"].iter().enumerate() {
        // D-term LPF1
        if let (Some(filter_type_str), Some(cutoff_str)) = (
            header_map.get("dterm_filter_type"),
            header_map.get(&format!("dterm_lowpass_hz_{axis_name}")),
        ) {
            if let (Ok(filter_type_num), Ok(cutoff_hz)) =
                (filter_type_str.parse::<u32>(), cutoff_str.parse::<f64>())
            {
                if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                    if cutoff_hz > 0.0 {
                        // Parse Q-factor if this is a BIQUAD filter
                        let q_factor = if filter_type == FilterType::Biquad {
                            header_map
                                .get(&format!("dterm_lowpass_q_{axis_name}"))
                                .or_else(|| header_map.get("dterm_lowpass_q"))
                                .and_then(|s| s.parse::<f64>().ok())
                        } else {
                            None
                        };

                        // Only add if cutoff > 0
                        config.dterm[axis_idx].lpf1 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }

        // D-term LPF2
        if let (Some(filter_type_str), Some(cutoff_str)) = (
            header_map.get("dterm_filter2_type"),
            header_map.get(&format!("dterm_lowpass2_hz_{axis_name}")),
        ) {
            if let (Ok(filter_type_num), Ok(cutoff_hz)) =
                (filter_type_str.parse::<u32>(), cutoff_str.parse::<f64>())
            {
                if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                    if cutoff_hz > 0.0 {
                        // Parse Q-factor if this is a BIQUAD filter
                        let q_factor = if filter_type == FilterType::Biquad {
                            header_map
                                .get(&format!("dterm_lowpass2_q_{axis_name}"))
                                .or_else(|| header_map.get("dterm_lowpass2_q"))
                                .and_then(|s| s.parse::<f64>().ok())
                        } else {
                            None
                        };

                        // Only add if cutoff > 0
                        config.dterm[axis_idx].lpf2 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }

        // Gyro filters (per-axis)
        if let (Some(filter_type_str), Some(cutoff_str)) = (
            header_map.get("gyro_lowpass_type"),
            header_map.get(&format!("gyro_lowpass_hz_{axis_name}")),
        ) {
            if let (Ok(filter_type_num), Ok(cutoff_hz)) =
                (filter_type_str.parse::<u32>(), cutoff_str.parse::<f64>())
            {
                if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                    if cutoff_hz > 0.0 {
                        // Parse Q-factor if this is a BIQUAD filter
                        let q_factor = if filter_type == FilterType::Biquad {
                            header_map
                                .get(&format!("gyro_lowpass_q_{axis_name}"))
                                .or_else(|| header_map.get("gyro_lowpass_q"))
                                .and_then(|s| s.parse::<f64>().ok())
                        } else {
                            None
                        };

                        // Only add if cutoff > 0
                        config.gyro[axis_idx].lpf1 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }

        // Gyro LPF2 (per-axis)
        if let (Some(filter_type_str), Some(cutoff_str)) = (
            header_map.get("gyro_lowpass2_type"),
            header_map.get(&format!("gyro_lowpass2_hz_{axis_name}")),
        ) {
            if let (Ok(filter_type_num), Ok(cutoff_hz)) =
                (filter_type_str.parse::<u32>(), cutoff_str.parse::<f64>())
            {
                if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                    if cutoff_hz > 0.0 {
                        // Parse Q-factor if this is a BIQUAD filter
                        let q_factor = if filter_type == FilterType::Biquad {
                            header_map
                                .get(&format!("gyro_lowpass2_q_{axis_name}"))
                                .or_else(|| header_map.get("gyro_lowpass2_q"))
                                .and_then(|s| s.parse::<f64>().ok())
                        } else {
                            None
                        };

                        // Only add if cutoff > 0
                        config.gyro[axis_idx].lpf2 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }
    }

    config
}

/// Parse Betaflight unified filter configuration from headers
pub fn parse_betaflight_filters(headers: &[(String, String)]) -> AllFilterConfigs {
    let header_map: HashMap<String, String> = headers
        .iter()
        .map(|(k, v)| (k.trim().to_lowercase(), v.trim().to_string()))
        .collect();

    let mut config = AllFilterConfigs::default();

    // Parse D-term filters (unified across all axes)
    if let Some(filter_type_str) = header_map.get("dterm_lpf1_type") {
        if let Ok(filter_type_num) = filter_type_str.parse::<u32>() {
            if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                // Check for static vs dynamic mode
                let static_cutoff = header_map
                    .get("dterm_lpf1_static_hz")
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(0.0);

                let dynamic_cutoffs = header_map
                    .get("dterm_lpf1_dyn_hz")
                    .map(|s| parse_dynamic_cutoffs(s))
                    .unwrap_or((0.0, 0.0));

                let expo = header_map
                    .get("dterm_lpf1_dyn_expo")
                    .and_then(|s| s.parse::<u32>().ok())
                    .unwrap_or(0);

                // Parse Q-factor if BIQUAD
                let q_factor = if filter_type == FilterType::Biquad {
                    header_map
                        .get("dterm_lowpass_q")
                        .and_then(|s| s.parse::<f64>().ok())
                } else {
                    None
                };

                // Apply to all axes - prioritize dynamic over static when both are present
                for axis_cfg in config.dterm.iter_mut().take(AXIS_NAMES.len()) {
                    if dynamic_cutoffs.0 > 0.0 && dynamic_cutoffs.1 > 0.0 {
                        // Dynamic mode takes precedence
                        axis_cfg.dynamic_lpf1 = Some(DynamicFilterConfig {
                            filter_type,
                            min_cutoff_hz: dynamic_cutoffs.0,
                            max_cutoff_hz: dynamic_cutoffs.1,
                            expo,
                            enabled: true,
                        });
                    } else if static_cutoff > 0.0 {
                        // Static mode fallback
                        axis_cfg.lpf1 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz: static_cutoff,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }
    }

    // Parse D-term LPF2 (static only)
    if let Some(filter_type_str) = header_map.get("dterm_lpf2_type") {
        if let Ok(filter_type_num) = filter_type_str.parse::<u32>() {
            if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                let static_cutoff = header_map
                    .get("dterm_lpf2_static_hz")
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(0.0);

                // Parse Q-factor if BIQUAD
                let q_factor = if filter_type == FilterType::Biquad {
                    header_map
                        .get("dterm_lowpass2_q")
                        .and_then(|s| s.parse::<f64>().ok())
                } else {
                    None
                };

                if static_cutoff > 0.0 {
                    for axis_cfg in config.dterm.iter_mut().take(AXIS_NAMES.len()) {
                        axis_cfg.lpf2 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz: static_cutoff,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }
    }

    // Parse Gyro filters (similar pattern)
    if let Some(filter_type_str) = header_map.get("gyro_lpf1_type") {
        if let Ok(filter_type_num) = filter_type_str.parse::<u32>() {
            if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                let static_cutoff = header_map
                    .get("gyro_lpf1_static_hz")
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(0.0);

                let dynamic_cutoffs = header_map
                    .get("gyro_lpf1_dyn_hz")
                    .map(|s| parse_dynamic_cutoffs(s))
                    .unwrap_or((0.0, 0.0));

                let expo = header_map
                    .get("gyro_lpf1_dyn_expo")
                    .and_then(|s| s.parse::<u32>().ok())
                    .unwrap_or(0);

                // Parse Q-factor if BIQUAD
                let q_factor = if filter_type == FilterType::Biquad {
                    header_map
                        .get("gyro_lowpass_q")
                        .and_then(|s| s.parse::<f64>().ok())
                } else {
                    None
                };

                for axis_cfg in config.gyro.iter_mut().take(AXIS_NAMES.len()) {
                    if dynamic_cutoffs.0 > 0.0 && dynamic_cutoffs.1 > 0.0 {
                        // Dynamic mode takes precedence
                        axis_cfg.dynamic_lpf1 = Some(DynamicFilterConfig {
                            filter_type,
                            min_cutoff_hz: dynamic_cutoffs.0,
                            max_cutoff_hz: dynamic_cutoffs.1,
                            expo,
                            enabled: true,
                        });
                    } else if static_cutoff > 0.0 {
                        // Static mode fallback
                        axis_cfg.lpf1 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz: static_cutoff,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }
    }

    // Parse Gyro LPF2 (static only)
    if let Some(filter_type_str) = header_map.get("gyro_lpf2_type") {
        if let Ok(filter_type_num) = filter_type_str.parse::<u32>() {
            if let Some(filter_type) = FilterType::from_u32(filter_type_num) {
                let static_cutoff = header_map
                    .get("gyro_lpf2_static_hz")
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(0.0);

                // Parse Q-factor if BIQUAD
                let q_factor = if filter_type == FilterType::Biquad {
                    header_map
                        .get("gyro_lowpass2_q")
                        .and_then(|s| s.parse::<f64>().ok())
                } else {
                    None
                };

                if static_cutoff > 0.0 {
                    for axis_cfg in config.gyro.iter_mut().take(AXIS_NAMES.len()) {
                        axis_cfg.lpf2 = Some(FilterConfig {
                            filter_type,
                            cutoff_hz: static_cutoff,
                            q_factor,
                            enabled: true,
                        });
                    }
                }
            }
        }
    }

    config
}

/// Parse dynamic cutoff string format "min,max" into tuple
fn parse_dynamic_cutoffs(cutoff_str: &str) -> (f64, f64) {
    let parts: Vec<&str> = cutoff_str.trim_matches('"').split(',').collect();
    if parts.len() == 2 {
        let min_cutoff = parts[0].trim().parse::<f64>().unwrap_or(0.0);
        let max_cutoff = parts[1].trim().parse::<f64>().unwrap_or(0.0);
        (min_cutoff, max_cutoff)
    } else {
        (0.0, 0.0)
    }
}

/// Calculate per-stage PT1 cutoff frequency for IMUF filters
/// Uses Butterworth correction factors to achieve proper combined response
/// Based on IMU-F ptnFilter.c: Adj_f_cut = (float)f_cut * ScaleF[filter->order - 1]
///
/// Example: For PT2 at 90 Hz combined:
/// - Each PT1 stage runs at 90 * 1.554 = 140 Hz
/// - Two stages at 140 Hz produce -3dB at 90 Hz combined
fn calculate_ptn_per_stage_cutoff(configured_cutoff_hz: f64, ptn_order: u32) -> f64 {
    // PTn cutoff correction factors from Betaflight/EmuFlight source
    // Formula: 1 / sqrt(2^(1/n) - 1) where n is the filter order
    const PTN_SCALE_FACTORS: [f64; 4] = [
        1.0,         // PT1 (order 1) - no correction needed
        1.553773974, // PT2 (order 2) - CUTOFF_CORRECTION_PT2
        1.961459177, // PT3 (order 3) - CUTOFF_CORRECTION_PT3
        2.298959223, // PT4 (order 4) - CUTOFF_CORRECTION_PT4
    ];

    let scale_factor = match ptn_order {
        1..=4 => PTN_SCALE_FACTORS[(ptn_order - 1) as usize],
        _ => 1.553773974, // Default to PT2 scaling
    };

    configured_cutoff_hz * scale_factor
}

/// Parse IMUF filters from EmuFlight headers
/// Applies to ALL EmuFlight (both HELIOSPRING with external PTn and non-HELIO with pseudo-Kalman)
pub fn parse_imuf_filters_with_gyro_rate(
    headers: &[(String, String)],
    _gyro_rate_hz: Option<f64>,
) -> AllFilterConfigs {
    let header_map: HashMap<String, String> = headers
        .iter()
        .map(|(k, v)| (k.trim().to_lowercase(), v.trim().to_string()))
        .collect();

    let mut config = AllFilterConfigs::default();

    // Parse IMUF revision (applies to all axes)
    let imuf_revision = header_map
        .get("imuf_revision")
        .and_then(|s| s.parse::<u32>().ok());

    // Parse pseudo-Kalman filter window size (applies to ALL EmuFlight)
    // Non-HELIO: Controls windowed variance for Kalman measurement noise
    // HELIO: Controls windowing in external firmware
    let pseudo_kalman_w = header_map
        .get("imuf_w")
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|&w| w > 0.0);

    // Parse IMUF parameters for each axis
    let axis_names = ["roll", "pitch", "yaw"];
    for (axis_idx, axis_name) in axis_names.iter().enumerate() {
        // Parse per-axis lowpass cutoff frequencies (HELIOSPRING only)
        let lowpass_key = format!("imuf_lowpass_{}", axis_name);
        let lowpass_cutoff_hz = header_map
            .get(&lowpass_key)
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap_or(0.0);

        // Parse per-axis Q-factors (applies to ALL EmuFlight)
        // Scaled by 1000 in headers for easier integer tuning (100-16000 → 0.01-1.6)
        let q_key = format!("imuf_{}_q", axis_name);
        let q_factor_scaled = header_map
            .get(&q_key)
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap_or(0.0);
        let q_factor = if q_factor_scaled > 0.0 {
            (q_factor_scaled / 1000.0).clamp(0.01, 16.0) // Validate within safe range
        } else {
            0.0 // No Q-factor configured
        };

        // Parse PTn filter order (HELIOSPRING only)
        let ptn_order = header_map
            .get("imuf_ptn_order")
            .and_then(|s| s.parse::<u32>().ok())
            .unwrap_or(2); // Default to PT2 if not specified

        // Create IMUF filter config if we have valid Q-factor or lowpass cutoff
        // HELIOSPRING: Has both lowpass_cutoff_hz AND ptn_order
        // Non-HELIO: Only has Q-factor and IMUF_w (no external PTn filter to visualize)
        if lowpass_cutoff_hz > 0.0 || q_factor > 0.0 {
            let effective_cutoff_hz = if lowpass_cutoff_hz > 0.0 {
                calculate_ptn_per_stage_cutoff(lowpass_cutoff_hz, ptn_order)
            } else {
                0.0 // Non-HELIO: no PTn filter chain, just pseudo-Kalman parameters
            };

            config.gyro[axis_idx].imuf = Some(ImufFilterConfig {
                lowpass_cutoff_hz,
                ptn_order,
                q_factor,
                revision: imuf_revision,
                pseudo_kalman_w,
                effective_cutoff_hz,
                enabled: true,
            });
        }
    }

    config
}

/// Auto-detect firmware type and parse appropriate filter configuration
pub fn parse_filter_config(headers: &[(String, String)]) -> AllFilterConfigs {
    // Detect firmware type by looking for characteristic fields
    let header_map: HashMap<String, String> = headers
        .iter()
        .map(|(k, v)| (k.trim().to_lowercase(), v.trim().to_string()))
        .collect();

    // Check for IMUF patterns (Betaflight/EmuFlight with PTn Butterworth correction)
    let has_imuf_pattern = header_map.contains_key("imuf_lowpass_roll")
        || header_map.contains_key("imuf_lowpass_pitch")
        || header_map.contains_key("imuf_lowpass_yaw")
        || header_map.contains_key("imuf_ptn_order")
        || header_map.contains_key("imuf_revision");

    // Check for EmuFlight per-axis patterns
    let has_emuflight_pattern = header_map.contains_key("dterm_lowpass_hz_roll")
        || header_map.contains_key("gyro_lowpass_hz_roll");

    // Check for Betaflight unified patterns
    let has_betaflight_pattern = header_map.contains_key("dterm_lpf1_static_hz")
        || header_map.contains_key("dterm_lpf1_dyn_hz");

    let mut config = if has_emuflight_pattern {
        println!("\n--- Detected EmuFlight filter configuration (per-axis) ---");
        parse_emuflight_filters(headers)
    } else if has_betaflight_pattern {
        println!("\n--- Detected Betaflight filter configuration (unified) ---");
        parse_betaflight_filters(headers)
    } else {
        println!("\n--- No recognized filter configuration found in headers ---");
        AllFilterConfigs::default()
    };

    // Check for IMUF configuration and merge it with existing config
    // IMUF Q-factors and IMUF_w apply to ALL EmuFlight (both HELIOSPRING with external PTn and non-HELIO with pseudo-Kalman)
    let has_imuf_q_factors = header_map.contains_key("imuf_roll_q")
        || header_map.contains_key("imuf_pitch_q")
        || header_map.contains_key("imuf_yaw_q");

    if has_imuf_pattern || has_imuf_q_factors {
        if has_imuf_pattern {
            println!("Detected PTn filters with Butterworth correction (HELIOSPRING IMUF)");
        } else {
            println!("Detected EmuFlight pseudo-Kalman filter (Q-factors and window)");
        }

        // Extract gyro rate for sample rate correction calculations
        let gyro_rate_hz = extract_gyro_rate(Some(headers));
        let imuf_config = parse_imuf_filters_with_gyro_rate(headers, gyro_rate_hz);

        // Merge IMUF filters into the existing configuration
        for axis_idx in 0..3 {
            if let Some(ref imuf_filter) = imuf_config.gyro[axis_idx].imuf {
                config.gyro[axis_idx].imuf = Some(imuf_filter.clone());
            }
        }
    } // Debug output: Print parsed filter configuration
    println!("Parsed filter configuration:");
    for (axis_idx, axis_name) in AXIS_NAMES.iter().enumerate() {
        println!("  {axis_name} Gyro Filters:");
        if let Some(ref lpf1) = config.gyro[axis_idx].lpf1 {
            if let Some(q) = lpf1.q_factor {
                println!(
                    "    LPF1: {} at {:.0} Hz (Q={:.2})",
                    lpf1.filter_type.name(),
                    lpf1.cutoff_hz,
                    q
                );
            } else {
                println!(
                    "    LPF1: {} at {:.0} Hz",
                    lpf1.filter_type.name(),
                    lpf1.cutoff_hz
                );
            }
        }
        if let Some(ref lpf2) = config.gyro[axis_idx].lpf2 {
            if let Some(q) = lpf2.q_factor {
                println!(
                    "    LPF2: {} at {:.0} Hz (Q={:.2})",
                    lpf2.filter_type.name(),
                    lpf2.cutoff_hz,
                    q
                );
            } else {
                println!(
                    "    LPF2: {} at {:.0} Hz",
                    lpf2.filter_type.name(),
                    lpf2.cutoff_hz
                );
            }
        }
        if let Some(ref dyn_lpf1) = config.gyro[axis_idx].dynamic_lpf1 {
            println!(
                "    Dyn LPF1: {} {:.0}-{:.0} Hz (expo: {})",
                dyn_lpf1.filter_type.name(),
                dyn_lpf1.min_cutoff_hz,
                dyn_lpf1.max_cutoff_hz,
                dyn_lpf1.expo
            );
        }
        if let Some(ref imuf) = config.gyro[axis_idx].imuf {
            if imuf.lowpass_cutoff_hz > 0.0 {
                // HELIOSPRING: Has external PTn filter chain
                let stage_count_text = match imuf.ptn_order {
                    1 => "One PT1",
                    2 => "Two PT1",
                    3 => "Three PT1",
                    4 => "Four PT1",
                    _ => "Two PT1",
                };

                let version_str = if let Some(rev) = imuf.revision {
                    format!("IMUF v{}", rev)
                } else {
                    "IMUF".to_string()
                };

                println!(
                    "    {}: {} Combined={:.0}Hz → per-stage={:.0}Hz (Q={:.1})",
                    version_str,
                    stage_count_text,
                    imuf.lowpass_cutoff_hz,
                    imuf.effective_cutoff_hz,
                    imuf.q_factor
                );

                // Show info about Butterworth correction
                let ptn_scaling_diff = imuf.effective_cutoff_hz - imuf.lowpass_cutoff_hz;

                if ptn_scaling_diff.abs() > 5.0 {
                    println!(
                        "      Note: Butterworth correction requires PT1 stages at {:.0}Hz to achieve {:.0}Hz combined response",
                        imuf.effective_cutoff_hz,
                        imuf.lowpass_cutoff_hz
                    );
                }
            } else {
                // Non-HELIOSPRING: Pseudo-Kalman only (no external PTn filter)
                println!(
                    "    EmuFlight Pseudo-Kalman: Q={:.1} (default Q=0.6)",
                    imuf.q_factor
                );
            }

            // Show pseudo-Kalman window info (applies to both HELIO and non-HELIO)
            if let Some(w) = imuf.pseudo_kalman_w {
                println!("      Pseudo-Kalman filter window size: {:.0} samples", w);
            }
        }
        if config.gyro[axis_idx].lpf1.is_none()
            && config.gyro[axis_idx].lpf2.is_none()
            && config.gyro[axis_idx].dynamic_lpf1.is_none()
            && config.gyro[axis_idx].imuf.is_none()
        {
            println!("    No gyro filters configured");
        }
    }

    config
}

/// Dynamic notch configuration including axis information
#[derive(Debug, Clone, Copy)]
pub struct DynamicNotchConfig {
    pub min_hz: f64,
    pub max_hz: f64,
    pub q_factor: f64,
    pub notch_count: u32,
    pub applies_to_yaw: bool, // false = Roll/Pitch only, true = Roll/Pitch/Yaw
}

/// Extract dynamic notch frequency range for graphical visualization
/// Returns DynamicNotchConfig if dynamic notch is configured
/// Handles Betaflight, Emuflight, and INAV formats
pub fn extract_dynamic_notch_range(
    header_metadata: Option<&[(String, String)]>,
) -> Option<DynamicNotchConfig> {
    let metadata = header_metadata?;

    // Detect firmware type from Firmware revision (reliable)
    // Firmware type is unreliable (Emuflight reports as "Cleanflight")
    let firmware_revision = metadata
        .iter()
        .find(|(key, _)| key == "Firmware revision")
        .map(|(_, value)| value.as_str())
        .unwrap_or("");

    // Helper to get metadata value
    let get_value = |key: &str| -> Option<String> {
        metadata
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
    };

    // Check firmware type by looking at revision string
    if firmware_revision.contains("Betaflight") {
        // Betaflight dynamic notch (always applies to all axes)
        if let Some(count_str) = get_value("dyn_notch_count") {
            if let Ok(count) = count_str.parse::<u32>() {
                if count > 0 {
                    let min_hz = get_value("dyn_notch_min_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(90.0);
                    let max_hz = get_value("dyn_notch_max_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(800.0);
                    let q = get_value("dyn_notch_q")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(400.0);

                    return Some(DynamicNotchConfig {
                        min_hz,
                        max_hz,
                        q_factor: q,
                        notch_count: count,
                        applies_to_yaw: true, // Betaflight always applies to all axes
                    });
                }
            }
        }
    } else if firmware_revision.contains("EmuFlight") || firmware_revision.contains("Emuflight") {
        // Emuflight dynamic gyro notch with axis configuration
        if let Some(count_str) = get_value("dynamic_gyro_notch_count") {
            if let Ok(count) = count_str.parse::<u32>() {
                if count > 0 {
                    let min_hz = get_value("dynamic_gyro_notch_min_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(150.0);
                    let max_hz = get_value("dynamic_gyro_notch_max_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(400.0);
                    let q = get_value("dynamic_gyro_notch_q")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(500.0);

                    // Check axis configuration: 0 = RP only, 1 = RPY (default if not present)
                    let applies_to_yaw = get_value("dynamic_gyro_notch_axis")
                        .and_then(|s| s.parse::<u32>().ok())
                        .map(|axis| axis == 1) // 1 means RPY
                        .unwrap_or(true); // Default to RPY if not specified

                    return Some(DynamicNotchConfig {
                        min_hz,
                        max_hz,
                        q_factor: q,
                        notch_count: count,
                        applies_to_yaw,
                    });
                }
            }
        }
    } else if firmware_revision.contains("INAV") {
        // INAV dynamic gyro notch (limited configuration)
        // Check for Q factor to determine if dynamic notch is enabled
        if let Some(q_str) = get_value("dynamicGyroNotchQ") {
            if let Ok(q) = q_str.parse::<f64>() {
                if q > 0.0 {
                    let min_hz = get_value("dynamicGyroNotchMinHz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(150.0);

                    // INAV doesn't specify max_hz or count in headers
                    // Use reasonable defaults based on INAV behavior
                    let max_hz = 350.0; // INAV typical max
                    let count = 1; // INAV typically uses 1 notch

                    return Some(DynamicNotchConfig {
                        min_hz,
                        max_hz,
                        q_factor: q,
                        notch_count: count,
                        applies_to_yaw: true, // INAV applies to all axes
                    });
                }
            }
        }
    }

    None
}

/// Extract D-term dynamic notch configuration for Emuflight
/// Returns DynamicNotchConfig if D-term dynamic notch is enabled
/// Only Emuflight has D-term dynamic notch feature
/// Requires both dterm_dyn_notch_enable=1 AND dynamic_gyro_notch_count>0
pub fn extract_dterm_dynamic_notch_range(
    header_metadata: Option<&[(String, String)]>,
) -> Option<DynamicNotchConfig> {
    let metadata = header_metadata?;

    // Detect firmware type from Firmware revision (reliable)
    let firmware_revision = metadata
        .iter()
        .find(|(key, _)| key == "Firmware revision")
        .map(|(_, value)| value.as_str())
        .unwrap_or("");

    // Helper to get metadata value
    let get_value = |key: &str| -> Option<String> {
        metadata
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
    };

    // Only Emuflight has D-term dynamic notch
    if firmware_revision.contains("EmuFlight") || firmware_revision.contains("Emuflight") {
        // Check if D-term dynamic notch is enabled
        if let Some(enable_str) = get_value("dterm_dyn_notch_enable") {
            if enable_str == "1" {
                // D-term dynamic notch requires gyro dynamic notch to be enabled
                if let Some(gyro_count_str) = get_value("dynamic_gyro_notch_count") {
                    if let Ok(gyro_count) = gyro_count_str.parse::<u32>() {
                        if gyro_count > 0 {
                            // Get D-term dynamic notch Q factor
                            let q = get_value("dterm_dyn_notch_q")
                                .and_then(|s| s.parse::<f64>().ok())
                                .unwrap_or(400.0);

                            // D-term dynamic notch uses same frequency range as gyro dynamic notch
                            let min_hz = get_value("dynamic_gyro_notch_min_hz")
                                .and_then(|s| s.parse::<f64>().ok())
                                .unwrap_or(150.0);
                            let max_hz = get_value("dynamic_gyro_notch_max_hz")
                                .and_then(|s| s.parse::<f64>().ok())
                                .unwrap_or(400.0);

                            // D-term dynamic notch uses same count as gyro
                            let count = gyro_count;

                            // Check axis configuration (same as gyro)
                            let applies_to_yaw = get_value("dynamic_gyro_notch_axis")
                                .and_then(|s| s.parse::<u32>().ok())
                                .map(|axis| axis == 1)
                                .unwrap_or(true);

                            return Some(DynamicNotchConfig {
                                min_hz,
                                max_hz,
                                q_factor: q,
                                notch_count: count,
                                applies_to_yaw,
                            });
                        }
                    }
                }
            }
        }
    }

    None
}

/// RPM filter configuration (Betaflight only)
#[derive(Debug, Clone)]
pub struct RpmFilterConfig {
    pub harmonics: u32,           // Number of harmonics (1-12)
    pub min_hz: f64,              // Minimum frequency to filter
    pub q_factor: f64,            // Q factor for notch width
    pub weights: Vec<f64>,        // Weight for each harmonic (0.0-1.0)
    pub motor_poles: Option<u32>, // Header motor_poles; needed to convert eRPM to Hz
    pub fade_range_hz: f64,       // Notch fades in over this range above min_hz
    #[allow(dead_code)] // Reserved for future use
    pub lpf_hz: f64, // RPM signal LPF
}

impl RpmFilterConfig {
    /// Per-harmonic weights as percentages, e.g. "100%, 100%, 100%"
    pub fn weights_percent_display(&self) -> String {
        self.weights
            .iter()
            .map(|w| format!("{:.0}%", w * RATIO_TO_PERCENT))
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// Extract RPM filter configuration for Betaflight
/// Returns RpmFilterConfig if RPM filter is enabled
/// Only Betaflight has RPM filter feature
pub fn extract_rpm_filter_config(
    header_metadata: Option<&[(String, String)]>,
) -> Option<RpmFilterConfig> {
    let metadata = header_metadata?;

    // Detect firmware type from Firmware revision (reliable)
    let firmware_revision = metadata
        .iter()
        .find(|(key, _)| key == "Firmware revision")
        .map(|(_, value)| value.as_str())
        .unwrap_or("");

    // Helper to get metadata value
    let get_value = |key: &str| -> Option<String> {
        metadata
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
    };

    // Only Betaflight has RPM filter
    if firmware_revision.contains("Betaflight") {
        // Check if RPM filter is enabled
        if let Some(harmonics_str) = get_value("rpm_filter_harmonics") {
            if let Ok(harmonics) = harmonics_str.parse::<u32>() {
                if harmonics > 0 {
                    // Enforce documented upper bound (Betaflight max is 12)
                    let harmonics = harmonics.min(12);

                    // Parse configuration
                    let min_hz = get_value("rpm_filter_min_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(100.0);

                    let q = get_value("rpm_filter_q")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(500.0);

                    // Parse weights (comma-separated, e.g., "100,100,100")
                    let mut weights = if let Some(weights_str) = get_value("rpm_filter_weights") {
                        weights_str
                            .split(',')
                            .filter_map(|s| s.trim().parse::<f64>().ok())
                            .map(|w| (w / 100.0).clamp(0.0, 1.0)) // Convert 0-100 to 0.0-1.0 and clamp
                            .collect::<Vec<f64>>()
                    } else {
                        // Default: all harmonics have 100% weight
                        vec![1.0; harmonics as usize]
                    };

                    // Normalize vector length to match harmonics
                    match weights.len().cmp(&(harmonics as usize)) {
                        std::cmp::Ordering::Less => weights.resize(harmonics as usize, 1.0),
                        std::cmp::Ordering::Greater => weights.truncate(harmonics as usize),
                        std::cmp::Ordering::Equal => {}
                    }

                    let fade_range_hz = get_value("rpm_filter_fade_range_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(50.0);

                    let lpf_hz = get_value("rpm_filter_lpf_hz")
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(150.0);

                    let motor_poles =
                        get_value("motor_poles").and_then(|s| s.trim().parse::<u32>().ok());

                    return Some(RpmFilterConfig {
                        harmonics,
                        min_hz,
                        q_factor: q / 100.0, // Betaflight stores Q × 100
                        weights,
                        motor_poles,
                        fade_range_hz,
                        lpf_hz,
                    });
                }
            }
        }
    }

    None
}

/// Motor fundamental frequency (Hz) range over a flight, from logged eRPM.
/// Percentile convention: the sorted sample at index round(pct / 100 * (n - 1)), not nearest-rank.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RpmFundamentalHz {
    pub low: f64,    // RPM_FILTER_BAND_LOW_PERCENTILE of all motor samples (see below)
    pub median: f64, // Median of all motor samples
    pub high: f64,   // RPM_FILTER_BAND_HIGH_PERCENTILE of all motor samples (see below)
}

/// Motor fundamental frequency range (Hz) from logged eRPM, derived as Betaflight does:
/// Hz = eRPM_LSB * 100 / 60 / (poles / 2). Samples from all motors are pooled; non-finite and
/// non-positive samples are ignored. Returns None when no usable eRPM sample exists or
/// `motor_poles` is below 2.
pub fn rpm_fundamental_hz_from_erpm(
    erpms: impl Iterator<Item = f64>,
    motor_poles: u32,
) -> Option<RpmFundamentalHz> {
    if motor_poles < RPM_FILTER_MIN_MOTOR_POLES {
        return None;
    }
    let mut samples: Vec<f64> = erpms.filter(|v| v.is_finite() && *v > 0.0).collect();
    if samples.is_empty() {
        return None;
    }
    samples.sort_by(|a, b| a.total_cmp(b));
    let last = samples.len() - 1;
    let mid = samples.len() / 2;
    let median = if samples.len().is_multiple_of(PARITY_DIVISOR) {
        (samples[mid - 1] + samples[mid]) / 2.0
    } else {
        samples[mid]
    };
    let percentile = |pct: f64| samples[((pct / RATIO_TO_PERCENT) * last as f64).round() as usize];
    let pole_pairs = motor_poles as f64 / RPM_FILTER_POLES_PER_PAIR;
    let to_hz = |erpm_lsb: f64| {
        erpm_lsb * RPM_FILTER_ERPM_PER_LSB / RPM_FILTER_SECONDS_PER_MINUTE / pole_pairs
    };
    Some(RpmFundamentalHz {
        low: to_hz(percentile(RPM_FILTER_BAND_LOW_PERCENTILE)),
        median: to_hz(median),
        high: to_hz(percentile(RPM_FILTER_BAND_HIGH_PERCENTILE)),
    })
}

/// Legend text for the RPM filter overlay: `RPM Filter (N) @ median eRPM: W% @ Hz, ...`.
/// Lists all N configured harmonics at the median motor frequency; one above the plot range or
/// below `min_hz` is noted.
pub fn rpm_filter_legend_text(
    config: &RpmFilterConfig,
    base: &RpmFundamentalHz,
    max_freq_hz: f64,
    sample_rate_hz: f64,
) -> String {
    let items: Vec<String> = (1..=config.harmonics)
        .map(|harmonic| {
            let freq = base.median * harmonic as f64;
            let weight = config
                .weights
                .get(harmonic as usize - 1)
                .copied()
                .unwrap_or(1.0)
                * RATIO_TO_PERCENT;
            let note = if weight <= 0.0 {
                " (disabled)"
            } else if freq < config.min_hz && config.fade_range_hz > 0.0 {
                " (below min)"
            } else if freq
                .max(config.min_hz)
                .min(RPM_FILTER_MAX_HZ_RATE_FRACTION * sample_rate_hz)
                > max_freq_hz
            {
                // Same clamped center the curve generator draws
                " (off plot)"
            } else {
                ""
            };
            format!("{weight:.0}% @ {freq:.0}Hz{note}")
        })
        .collect();
    format!(
        "RPM Filter ({}) @ median eRPM: {}",
        config.harmonics,
        items.join(", ")
    )
}

/// Name of RPM filter harmonic `harmonic` (1 = motor fundamental) used by the heatmap legend,
/// console and report.
pub fn rpm_harmonic_label(harmonic: u32) -> String {
    format!("Harmonic {harmonic}")
}

/// Motor fundamental frequency ranges for a whole log: all motors pooled and each motor alone.
#[derive(Debug, Clone, PartialEq)]
pub struct RpmMotorFrequencies {
    pub pooled: RpmFundamentalHz,
    pub per_motor: Vec<Option<RpmFundamentalHz>>, // Indexed like LogRowData::motors
}

/// Motor frequency ranges from the log's eRPM channels and `motor_poles`. None when the config
/// has no pole count or no motor has a usable eRPM sample.
pub fn rpm_motor_frequencies(
    log_data: &[LogRowData],
    config: &RpmFilterConfig,
) -> Option<RpmMotorFrequencies> {
    let poles = config.motor_poles?;
    let motor_count = log_data.first().map(|r| r.motors.len()).unwrap_or(0);
    let pooled = rpm_fundamental_hz_from_erpm(
        log_data
            .iter()
            .flat_map(|r| r.erpms.iter().flatten().copied()),
        poles,
    )?;
    let per_motor = (0..motor_count)
        .map(|m| {
            rpm_fundamental_hz_from_erpm(
                log_data
                    .iter()
                    .filter_map(|r| r.erpms.get(m).copied().flatten()),
                poles,
            )
        })
        .collect();
    Some(RpmMotorFrequencies { pooled, per_motor })
}

/// Median motor fundamental (Hz) per throttle bin, pooling all motors. A sample lands in the
/// bin of its row's `setpoint[3]` (same binning as the throttle-frequency heatmap). A bin with
/// fewer than `min_samples` usable eRPM samples is `None`.
pub fn rpm_fundamental_hz_by_throttle_bin(
    log_data: &[LogRowData],
    motor_poles: u32,
    bin_count: usize,
    throttle_min: f64,
    throttle_max: f64,
    min_samples: usize,
) -> Vec<Option<f64>> {
    let bin_size = (throttle_max - throttle_min) / bin_count.max(1) as f64;
    if bin_count == 0 || bin_size <= 0.0 {
        return Vec::new();
    }
    let mut per_bin: Vec<Vec<f64>> = vec![Vec::new(); bin_count];
    for row in log_data {
        let Some(throttle) = row.setpoint[3] else {
            continue;
        };
        let idx = (((throttle - throttle_min) / bin_size).floor() as usize).min(bin_count - 1);
        per_bin[idx].extend(
            row.erpms
                .iter()
                .flatten()
                .copied()
                .filter(|v| v.is_finite() && *v > 0.0),
        );
    }
    per_bin
        .into_iter()
        .map(|samples| {
            if samples.len() < min_samples {
                return None;
            }
            rpm_fundamental_hz_from_erpm(samples.into_iter(), motor_poles).map(|f| f.median)
        })
        .collect()
}

/// RPM filter harmonic lines for the heatmap: harmonic `h` is drawn at `h` x the per-bin median
/// motor Hz (x) against the bin's throttle (y). A line breaks where a bin has no median or the
/// harmonic leaves the plot (`x > x_max`); runs under `min_points` points are dropped. Each run is tagged
/// with its harmonic number.
fn rpm_harmonic_overlay_lines(
    median_hz_by_bin: &[Option<f64>],
    throttle_centers: &[f64],
    harmonics: u32,
    x_max: f64,
    min_points: usize,
) -> Vec<(u32, Vec<(f64, f64)>)> {
    let mut lines = Vec::new();
    for harmonic in 1..=harmonics {
        let mut run: Vec<(f64, f64)> = Vec::new();
        let mut finish = |run: &mut Vec<(f64, f64)>| {
            if run.len() >= min_points {
                lines.push((harmonic, std::mem::take(run)));
            } else {
                run.clear();
            }
        };
        for (hz, &y) in median_hz_by_bin.iter().zip(throttle_centers) {
            match hz.map(|f| f * harmonic as f64) {
                Some(x) if x <= x_max => run.push((x, y)),
                _ => finish(&mut run),
            }
        }
        finish(&mut run);
    }
    lines
}

/// RPM harmonic overlay lines for a throttle-frequency heatmap, from the log's eRPM and the
/// header `motor_poles`. Empty when the config has no pole count or no bin has enough samples.
pub fn rpm_throttle_overlay_lines(
    log_data: &[LogRowData],
    config: &RpmFilterConfig,
    throttle_min: f64,
    throttle_max: f64,
    throttle_centers: &[f64],
    x_max: f64,
) -> Vec<(u32, Vec<(f64, f64)>)> {
    let Some(poles) = config.motor_poles else {
        return Vec::new();
    };
    let medians = rpm_fundamental_hz_by_throttle_bin(
        log_data,
        poles,
        throttle_centers.len(),
        throttle_min,
        throttle_max,
        RPM_HEATMAP_MIN_BIN_SAMPLES,
    );
    rpm_harmonic_overlay_lines(
        &medians,
        throttle_centers,
        config.harmonics,
        x_max,
        RPM_HEATMAP_MIN_LINE_POINTS,
    )
}

/// Console and report lines for the RPM filter motor frequencies as `(label, text)`.
/// Console prints `RPM Filter {label}: {text}`; the report prints `- **RPM Filter {label}:** {text}`.
pub fn rpm_motor_frequency_lines(
    config: &RpmFilterConfig,
    freqs: Option<&RpmMotorFrequencies>,
) -> Vec<(&'static str, String)> {
    let Some(f) = freqs else {
        return vec![(
            "motor frequency",
            "unavailable (log has no eRPM or motor_poles)".to_string(),
        )];
    };
    let range = |r: &RpmFundamentalHz, h: f64| {
        format!(
            "{:.0}–{:.0} Hz (median {:.0})",
            r.low * h,
            r.high * h,
            r.median * h
        )
    };
    let percentile_note = format!(
        "{RPM_FILTER_BAND_LOW_PERCENTILE:.0}th–{RPM_FILTER_BAND_HIGH_PERCENTILE:.0}th percentile"
    );
    let harmonics = (1..=config.harmonics)
        .map(|h| format!("{} {}", rpm_harmonic_label(h), range(&f.pooled, h as f64)))
        .collect::<Vec<_>>()
        .join(", ");
    let per_motor = f
        .per_motor
        .iter()
        .enumerate()
        .map(|(i, m)| match m {
            Some(r) => format!("M{i} {}", range(r, 1.0)),
            None => format!("M{i} no eRPM"),
        })
        .collect::<Vec<_>>()
        .join(", ");
    vec![
        (
            "motor frequency",
            format!(
                "{} (all motors, {percentile_note}, from eRPM)",
                range(&f.pooled, 1.0)
            ),
        ),
        ("harmonic ranges", harmonics),
        ("per-motor frequency", per_motor),
    ]
}

/// Magnitude response of Betaflight's RPM notch (`rpmNotchApply` in `common/filter.c`).
/// The filter is a state-variable band-pass subtracted from the input:
/// `out = in - (weight / Q) * BP(in)`, so H(s) = 1 - weight * (s / Q) / (s^2 + s / Q + 1)
/// with s = j * x. The digital form warps frequency as x = tan(pi f / fs) / tan(pi f0 / fs).
/// Returns the amplitude factor (1 - weight at the center, 1.0 far away). The -3 dB width of a
/// full-depth notch is about f0 / Q.
pub fn rpm_notch_response(
    frequency_hz: f64,
    notch_center_hz: f64,
    q_factor: f64,
    weight: f64,         // 0.0 to 1.0 (harmonic weight, after any fade)
    sample_rate_hz: f64, // Rate the filter runs at (gyro rate)
) -> f64 {
    if q_factor <= 0.0 || notch_center_hz <= 0.0 || sample_rate_hz <= 0.0 {
        return 1.0; // No filtering
    }
    let nyquist = sample_rate_hz / 2.0;
    if frequency_hz <= 0.0 || frequency_hz >= nyquist || notch_center_hz >= nyquist {
        return 1.0;
    }
    let weight = weight.clamp(0.0, 1.0);
    let x = (std::f64::consts::PI * frequency_hz / sample_rate_hz).tan()
        / (std::f64::consts::PI * notch_center_hz / sample_rate_hz).tan();
    let k = 1.0 / q_factor;
    let real = 1.0 - x * x;
    let numerator = real.powi(2) + (x * k * (1.0 - weight)).powi(2);
    let denominator = real.powi(2) + (x * k).powi(2);
    (numerator / denominator).sqrt()
}

/// Rate (Hz) the Betaflight RPM filter runs at: the PID loop rate, which is the gyro rate
/// divided by `pid_process_denom` (`gyro_init.c`: `targetLooptime = activePidLoopDenom * 1e6 /
/// sampleRateHz`; `rpmFilterInit(.., gyro.targetLooptime)`). The blackbox `looptime` header is the
/// gyro sample period in microseconds. Falls back to `RPM_FILTER_DEFAULT_GYRO_RATE_HZ` when
/// `looptime` is absent or unusable, and to a denominator of 1 when `pid_process_denom` is.
pub fn extract_rpm_filter_rate(header_metadata: Option<&[(String, String)]>) -> f64 {
    let get = |key: &str| -> Option<f64> {
        header_metadata?
            .iter()
            .find(|(k, _)| k == key)
            .and_then(|(_, v)| v.trim().parse::<f64>().ok())
    };
    let gyro_rate = get("looptime")
        .filter(|us| *us > 0.0)
        .map(|us| RPM_FILTER_MICROSECONDS_PER_SECOND / us)
        .unwrap_or(RPM_FILTER_DEFAULT_GYRO_RATE_HZ);
    let denom = get("pid_process_denom")
        .filter(|d| *d >= 1.0)
        .unwrap_or(1.0);
    gyro_rate / denom
}

/// Generate RPM filter notch curves for all harmonics
/// Returns vector of (harmonic_num, label, curve_points, center_hz) for each harmonic
pub fn generate_rpm_filter_curves(
    config: &RpmFilterConfig,
    motor_base_hz: f64,
    max_frequency_hz: f64,
    num_points: usize,
    sample_rate_hz: f64,
) -> Vec<RpmFilterCurveData> {
    if num_points < 2 {
        return Vec::new();
    }

    let mut curves = Vec::new();
    let min_plot_start =
        (config.min_hz * RPM_FILTER_CURVE_START_FRACTION).max(RPM_FILTER_CURVE_START_FLOOR_HZ);

    // Early return if no valid frequency span
    if max_frequency_hz <= min_plot_start {
        return curves;
    }

    // Calculate log spacing once (constant for all harmonics)
    let log_start = min_plot_start.ln();
    let log_end = max_frequency_hz.ln();
    let log_step = (log_end - log_start) / (num_points - 1) as f64;

    // Generate curve for each harmonic
    for harmonic_num in 1..=config.harmonics {
        let raw_freq = motor_base_hz * harmonic_num as f64;

        // Betaflight clamps the notch center to [min_hz, 0.48 * rate] (rpm_filter.c), so the
        // display range check uses the clamped center. A center below min_hz sits at min_hz
        // with weight 0 when the fade range is positive (the margin is 0), and with full weight
        // when the fade range is 0.
        let harmonic_freq = raw_freq
            .max(config.min_hz)
            .min(RPM_FILTER_MAX_HZ_RATE_FRACTION * sample_rate_hz);
        if harmonic_freq > max_frequency_hz {
            continue;
        }

        // Get weight for this harmonic (default to 1.0 if not specified)
        let weight = config
            .weights
            .get((harmonic_num - 1) as usize)
            .copied()
            .unwrap_or(1.0);

        // Skip if weight is zero (disabled harmonic)
        if weight <= 0.0 {
            continue;
        }
        // Betaflight fades a notch out as its center approaches min_hz (rpm_filter.c)
        let margin_hz = harmonic_freq - config.min_hz;
        let weight = if config.fade_range_hz > 0.0 && margin_hz < config.fade_range_hz {
            weight * (margin_hz / config.fade_range_hz)
        } else {
            weight
        };

        // Generate curve points
        let mut curve_points = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let freq = (log_start + i as f64 * log_step).exp();
            let response =
                rpm_notch_response(freq, harmonic_freq, config.q_factor, weight, sample_rate_hz);
            curve_points.push((freq, response));
        }

        // Keep only the span where the notch attenuates by at least the threshold, plus one
        // neighbor each side so the line joins the flat level. No such span: nothing to draw.
        let threshold = 1.0 - RPM_FILTER_CURVE_MIN_ATTENUATION;
        let (Some(first), Some(last)) = (
            curve_points.iter().position(|p| p.1 <= threshold),
            curve_points.iter().rposition(|p| p.1 <= threshold),
        ) else {
            continue;
        };
        let curve_points =
            curve_points[first.saturating_sub(1)..=(last + 1).min(num_points - 1)].to_vec();

        // Create label
        let label = format!(
            "RPM {} @ {:.0}Hz",
            rpm_harmonic_label(harmonic_num),
            harmonic_freq
        );

        curves.push((harmonic_num, label, curve_points, harmonic_freq));
    }

    curves
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn harmonics_are_multiples_of_median_and_clipped() {
        let medians = [Some(100.0), Some(120.0), Some(140.0)];
        let y = [10.0, 20.0, 30.0];
        let lines = rpm_harmonic_overlay_lines(&medians, &y, 3, 300.0, 2);
        // Harmonics 1 and 2 stay on plot; harmonic 3 has one on-plot point (300 at bin 0), so it is dropped.
        assert_eq!(lines[0].0, 1);
        assert_eq!(lines[1].0, 2);
        assert_eq!(
            lines[0].1,
            vec![(100.0, 10.0), (120.0, 20.0), (140.0, 30.0)]
        );
        assert_eq!(
            lines[1].1,
            vec![(200.0, 10.0), (240.0, 20.0), (280.0, 30.0)]
        );
        assert_eq!(lines.len(), 2);
        // The plot limit is inclusive: points at 270 and exactly 300 Hz form a run that ends at 300.
        let edge =
            rpm_harmonic_overlay_lines(&[Some(90.0), Some(100.0), Some(110.0)], &y, 3, 300.0, 2);
        assert_eq!(edge[2].0, 3);
        assert_eq!(edge[2].1, vec![(270.0, 10.0), (300.0, 20.0)]);
        // A higher minimum drops the three-point runs too.
        assert!(rpm_harmonic_overlay_lines(&medians, &y, 3, 300.0, 4).is_empty());
    }

    #[test]
    fn missing_bin_splits_the_line() {
        let medians = [Some(100.0), None, Some(110.0), Some(120.0)];
        let y = [1.0, 2.0, 3.0, 4.0];
        let lines = rpm_harmonic_overlay_lines(&medians, &y, 1, 500.0, 2);
        assert_eq!(lines, vec![(1, vec![(110.0, 3.0), (120.0, 4.0)])]);
    }

    #[test]
    fn rpm_harmonic_label_is_shared_by_report_text() {
        let label = rpm_harmonic_label(2);
        assert_eq!(label, "Harmonic 2");
        let cfg = RpmFilterConfig {
            harmonics: 2,
            min_hz: 100.0,
            q_factor: 5.0,
            weights: vec![1.0, 1.0],
            motor_poles: Some(12),
            fade_range_hz: 50.0,
            lpf_hz: 150.0,
        };
        let freqs = RpmMotorFrequencies {
            pooled: RpmFundamentalHz {
                low: 100.0,
                median: 200.0,
                high: 300.0,
            },
            per_motor: vec![],
        };
        let lines = rpm_motor_frequency_lines(&cfg, Some(&freqs));
        assert!(lines
            .iter()
            .any(|(k, text)| *k == "harmonic ranges" && text.contains(&label)));
    }

    #[test]
    fn test_rpm_fundamental_hz_by_throttle_bin() {
        // Throttle 0..1000 in 10 bins of 100: setpoint 50 -> bin 0, 950 -> bin 9, 1000 clamps to bin 9
        let row = |throttle: f64, erpms: Vec<Option<f64>>| LogRowData {
            setpoint: [None, None, None, Some(throttle)],
            erpms,
            ..Default::default()
        };
        let log = vec![
            row(50.0, vec![Some(840.0), Some(840.0)]),
            row(60.0, vec![Some(840.0), None]),
            row(950.0, vec![Some(1680.0), Some(1680.0)]),
            row(1000.0, vec![Some(1680.0), Some(0.0)]),
            row(450.0, vec![Some(500.0)]), // below min_samples: bin 4 stays None
        ];
        let bins = rpm_fundamental_hz_by_throttle_bin(&log, 12, 10, 0.0, 1000.0, 3);
        assert_eq!(bins.len(), 10);
        // Spec: 12 poles, 840 LSB = 233.333 Hz; 1680 LSB = 466.667 Hz
        assert!((bins[0].unwrap() - 233.333).abs() < 0.01);
        assert!((bins[9].unwrap() - 466.667).abs() < 0.01);
        assert!(bins[4].is_none() && bins[1].is_none());
        assert!(rpm_fundamental_hz_by_throttle_bin(&log, 12, 0, 0.0, 1000.0, 1).is_empty());

        // One usable sample plus zeros does not meet the minimum; the same count of usable samples does.
        let mut zeros = vec![row(550.0, vec![Some(840.0)])];
        zeros.extend((0..19).map(|_| row(550.0, vec![Some(0.0)])));
        assert!(rpm_fundamental_hz_by_throttle_bin(&zeros, 12, 10, 0.0, 1000.0, 20)[5].is_none());
        let valid: Vec<_> = (0..20).map(|_| row(550.0, vec![Some(840.0)])).collect();
        assert!(rpm_fundamental_hz_by_throttle_bin(&valid, 12, 10, 0.0, 1000.0, 20)[5].is_some());
    }

    #[test]
    fn test_rpm_fundamental_hz_from_erpm() {
        let hz_per_lsb_12_poles = 100.0 / 60.0 / 6.0; // 12 poles = 6 pole pairs
                                                      // Spec: 840 LSB = 84000 eRPM = 14000 RPM = 233.33 Hz; constant input gives zero spread
        let r = rpm_fundamental_hz_from_erpm([840.0, 840.0, 840.0].into_iter(), 12).unwrap();
        assert!((r.median - 233.333).abs() < 0.01);
        assert_eq!((r.low, r.high), (r.median, r.median));
        // 14 poles = 7 pairs; 1000 LSB = 100000 eRPM -> 238.095 Hz
        let r = rpm_fundamental_hz_from_erpm([1000.0].into_iter(), 14).unwrap();
        assert!((r.median - 238.095).abs() < 0.01);
        // 1..=101 LSB pooled: sorted index round(pct/100*(n-1)) gives p10 = 11, median = 51, p90 = 91
        let r = rpm_fundamental_hz_from_erpm((1..=101).map(f64::from), 12).unwrap();
        assert!((r.low - 11.0 * hz_per_lsb_12_poles).abs() < 1e-9);
        assert!((r.median - 51.0 * hz_per_lsb_12_poles).abs() < 1e-9);
        assert!((r.high - 91.0 * hz_per_lsb_12_poles).abs() < 1e-9);
        // 1..=10 LSB: p10 index round(0.9) = 1 -> 2, p90 index round(8.1) = 8 -> 9, median (5 + 6) / 2
        let r = rpm_fundamental_hz_from_erpm((1..=10).map(f64::from), 12).unwrap();
        assert!((r.low - 2.0 * hz_per_lsb_12_poles).abs() < 1e-9);
        assert!((r.median - 5.5 * hz_per_lsb_12_poles).abs() < 1e-9);
        assert!((r.high - 9.0 * hz_per_lsb_12_poles).abs() < 1e-9);
        // Even count averages the middle pair; zero, negative and NaN samples are ignored
        let r = rpm_fundamental_hz_from_erpm([0.0, -5.0, f64::NAN, 600.0, 1000.0].into_iter(), 12)
            .unwrap();
        assert!((r.median - 800.0 * hz_per_lsb_12_poles).abs() < 0.01);
        // No usable samples, or fewer than 2 poles: no frequency, no guess
        assert!(rpm_fundamental_hz_from_erpm(std::iter::empty(), 12).is_none());
        assert!(rpm_fundamental_hz_from_erpm([0.0, 0.0].into_iter(), 12).is_none());
        assert!(rpm_fundamental_hz_from_erpm([840.0].into_iter(), 1).is_none());
        assert!(rpm_fundamental_hz_from_erpm([840.0].into_iter(), 0).is_none());
    }

    fn test_rpm_config(harmonics: u32, weights: Vec<f64>) -> RpmFilterConfig {
        RpmFilterConfig {
            harmonics,
            min_hz: 100.0,
            q_factor: 5.0,
            weights,
            motor_poles: Some(12),
            fade_range_hz: 50.0,
            lpf_hz: 150.0,
        }
    }

    #[test]
    fn test_rpm_filter_legend_text() {
        let base = RpmFundamentalHz {
            low: 167.0,
            median: 229.0,
            high: 290.0,
        };
        let cfg = test_rpm_config(3, vec![1.0, 1.0, 1.0]);
        // Legend lists every configured harmonic at the median; no range
        assert_eq!(
            rpm_filter_legend_text(&cfg, &base, 503.0, 8000.0),
            "RPM Filter (3) @ median eRPM: 100% @ 229Hz, 100% @ 458Hz, 100% @ 687Hz (off plot)"
        );
        // Weight 0 stays in the legend as 0%
        let cfg0 = test_rpm_config(3, vec![1.0, 0.0, 1.0]);
        assert!(
            rpm_filter_legend_text(&cfg0, &base, 503.0, 8000.0).contains("0% @ 458Hz (disabled)")
        );
        // Median below min_hz is noted
        let low = RpmFundamentalHz {
            low: 20.0,
            median: 30.0,
            high: 40.0,
        };
        let cfg2 = test_rpm_config(2, vec![1.0, 1.0]);
        assert_eq!(
            rpm_filter_legend_text(&cfg2, &low, 503.0, 8000.0),
            "RPM Filter (2) @ median eRPM: 100% @ 30Hz (below min), 100% @ 60Hz (below min)"
        );
    }

    #[test]
    fn test_rpm_filter_legend_uses_clamped_center() {
        // H3 raw 2100 Hz is clamped to 0.48 * 4000 = 1920 Hz, inside the 2000 Hz plot: not off plot
        let cfg = test_rpm_config(3, vec![1.0, 1.0, 1.0]);
        let base = RpmFundamentalHz {
            low: 600.0,
            median: 700.0,
            high: 800.0,
        };
        let text = rpm_filter_legend_text(&cfg, &base, 2000.0, 4000.0);
        assert!(
            text.contains("100% @ 2100Hz,") || text.ends_with("100% @ 2100Hz"),
            "{text}"
        );
        assert!(!text.contains("off plot"), "{text}");
        // Below the clamp the raw center decides: a plot limit under 1920 Hz marks it off plot
        assert!(rpm_filter_legend_text(&cfg, &base, 1500.0, 4000.0).contains("2100Hz (off plot)"));
        // Fade 0: a median below min_hz is clamped to min_hz at full weight, not "below min"
        let mut no_fade = test_rpm_config(1, vec![1.0]);
        no_fade.fade_range_hz = 0.0;
        let low = RpmFundamentalHz {
            low: 80.0,
            median: 90.0,
            high: 95.0,
        };
        assert_eq!(
            rpm_filter_legend_text(&no_fade, &low, 503.0, 8000.0),
            "RPM Filter (1) @ median eRPM: 100% @ 90Hz"
        );
    }

    #[test]
    fn test_rpm_motor_frequency_lines() {
        let cfg = test_rpm_config(2, vec![1.0, 1.0]);
        let f = RpmMotorFrequencies {
            pooled: RpmFundamentalHz {
                low: 167.0,
                median: 229.0,
                high: 290.0,
            },
            per_motor: vec![
                Some(RpmFundamentalHz {
                    low: 160.0,
                    median: 220.0,
                    high: 280.0,
                }),
                None,
            ],
        };
        let lines = rpm_motor_frequency_lines(&cfg, Some(&f));
        assert_eq!(lines[0].0, "motor frequency");
        assert_eq!(
            lines[0].1,
            "167–290 Hz (median 229) (all motors, 10th–90th percentile, from eRPM)"
        );
        assert_eq!(
            lines[1].1,
            "Harmonic 1 167–290 Hz (median 229), Harmonic 2 334–580 Hz (median 458)"
        );
        assert_eq!(lines[2].1, "M0 160–280 Hz (median 220), M1 no eRPM");
        let none = rpm_motor_frequency_lines(&cfg, None);
        assert_eq!(none.len(), 1);
        assert!(none[0].1.contains("unavailable"));
    }

    /// Betaflight `rpmNotchUpdate` / `rpmNotchApply` difference equations, run on a sine.
    fn simulate_bf_rpm_notch_gain(f0: f64, f: f64, q: f64, weight: f64, fs: f64) -> f64 {
        let dt = 1.0 / fs;
        let ff = (std::f64::consts::PI * f0 * dt).tan();
        let qq = 1.0 / q;
        let a1 = 1.0 / (1.0 + ff * (ff + qq));
        let a2 = ff * a1;
        let wq = qq * weight;
        let (mut ic1, mut ic2) = (0.0f64, 0.0f64);
        let n = (fs * 3.0) as usize;
        let mut peak = 0.0f64;
        for i in 0..n {
            let input = (2.0 * std::f64::consts::PI * f * i as f64 * dt).sin();
            let v3 = input - ic2;
            let v1 = a1 * ic1 + a2 * v3;
            let v2 = ic2 + ff * v1;
            ic1 = 2.0 * v1 - ic1;
            ic2 = 2.0 * v2 - ic2;
            let out = input - wq * v1;
            if i > n / 2 {
                peak = peak.max(out.abs());
            }
        }
        peak
    }

    #[test]
    fn test_rpm_notch_response_matches_betaflight_filter() {
        let fs = 8000.0;
        for &(f0, q, w) in &[(229.0, 5.0, 1.0), (458.0, 5.0, 0.6), (150.0, 3.0, 1.0)] {
            for &f in &[
                0.5 * f0,
                0.9 * f0,
                0.97 * f0,
                f0,
                1.03 * f0,
                1.1 * f0,
                1.5 * f0,
            ] {
                let model = rpm_notch_response(f, f0, q, w, fs);
                let simulated = simulate_bf_rpm_notch_gain(f0, f, q, w, fs);
                assert!(
                    (model - simulated).abs() < 0.02,
                    "f0 {f0} f {f} q {q} w {w}: model {model} simulated {simulated}"
                );
            }
        }
    }

    #[test]
    fn test_rpm_notch_response_shape() {
        let fs = 8000.0;
        // Center depth is 1 - weight; weight 0 is flat
        assert!((rpm_notch_response(229.0, 229.0, 5.0, 1.0, fs)).abs() < 1e-9);
        assert!((rpm_notch_response(229.0, 229.0, 5.0, 0.4, fs) - 0.6).abs() < 1e-9);
        assert!((rpm_notch_response(229.0, 229.0, 5.0, 0.0, fs) - 1.0).abs() < 1e-9);
        // Full-depth -3 dB half-width is about f0 / (2 Q): 22.9 Hz for 229 Hz at Q 5
        let edge = rpm_notch_response(229.0 + 229.0 / 10.0, 229.0, 5.0, 1.0, fs);
        assert!(
            (edge - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.03,
            "edge {edge}"
        );
        // Invalid input is flat
        assert_eq!(rpm_notch_response(100.0, 229.0, 0.0, 1.0, fs), 1.0);
        assert_eq!(rpm_notch_response(100.0, 229.0, 5.0, 1.0, 0.0), 1.0);
        assert_eq!(rpm_notch_response(5000.0, 229.0, 5.0, 1.0, fs), 1.0);
    }

    #[test]
    fn test_generate_rpm_filter_curves_fade_and_clamp() {
        let cfg = test_rpm_config(2, vec![1.0, 1.0]); // min 100, fade 50, Q 5
        let fs = 8000.0;
        let depth_at = |curves: &[RpmFilterCurveData], harmonic: u32, center: f64| -> f64 {
            let c = curves.iter().find(|c| c.0 == harmonic).expect("curve");
            c.2.iter()
                .min_by(|a, b| (a.0 - center).abs().total_cmp(&(b.0 - center).abs()))
                .map(|p| 1.0 - p.1)
                .unwrap()
        };
        // Center 125 Hz is 25 Hz above min_hz: half of the 50 Hz fade range, so half depth
        let curves = generate_rpm_filter_curves(&cfg, 125.0, 500.0, 4000, fs);
        assert!((depth_at(&curves, 1, 125.0) - 0.5).abs() < 0.02);
        // Center 250 Hz is past the fade range: full depth
        assert!(depth_at(&curves, 2, 250.0) > 0.98);
        // Fundamental exactly at min_hz: weight fades to 0, nothing attenuates, no curve
        let at_min = generate_rpm_filter_curves(&cfg, 100.0, 500.0, 4000, fs);
        assert!(at_min.iter().all(|c| c.0 != 1));
        // Below min_hz with a positive fade range: the weight fades to 0, no curve
        assert!(generate_rpm_filter_curves(&cfg, 90.0, 500.0, 4000, fs)
            .iter()
            .all(|c| c.0 != 1));
        // Below min_hz with fade 0: Betaflight clamps the center to min_hz at full weight
        let mut no_fade = test_rpm_config(1, vec![1.0]);
        no_fade.fade_range_hz = 0.0;
        let curves = generate_rpm_filter_curves(&no_fade, 90.0, 500.0, 4000, fs);
        assert_eq!(curves[0].3, 100.0);
        assert!(depth_at(&curves, 1, 100.0) > 0.98);
    }

    #[test]
    fn test_generate_rpm_filter_curves_trimmed_to_attenuated_span() {
        let cfg = test_rpm_config(1, vec![1.0]); // Q 5
        let curves = generate_rpm_filter_curves(&cfg, 229.0, 503.0, 4000, 8000.0);
        let points = &curves[0].2;
        let (first, last) = (points.first().unwrap(), points.last().unwrap());
        // |H| = 0.95 at x = f / f0 where (1 - x^2) = 0.6086 x (Q 5): x = 0.741, and 1 / 0.741
        // on the high side (the response is symmetric in log frequency)
        assert!((first.0 - 229.0 * 0.741).abs() < 6.0, "start {}", first.0);
        assert!((last.0 - 229.0 / 0.741).abs() < 6.0, "end {}", last.0);
        // Every interior point attenuates; the two ends sit just above the threshold
        let threshold = 1.0 - RPM_FILTER_CURVE_MIN_ATTENUATION;
        assert!(points[1..points.len() - 1].iter().all(|p| p.1 <= threshold));
        // A harmonic weighted below the threshold attenuates less than 5%: no curve
        let weak = test_rpm_config(1, vec![0.04]);
        assert!(generate_rpm_filter_curves(&weak, 229.0, 503.0, 4000, 8000.0).is_empty());
    }

    #[test]
    fn test_extract_rpm_filter_rate() {
        let h = |pairs: &[(&str, &str)]| -> Vec<(String, String)> {
            pairs
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect()
        };
        // looptime 125 us = 8 kHz gyro, pid_process_denom 2: the filter runs at 4 kHz
        let m = h(&[
            ("looptime", "125"),
            ("gyro_sync_denom", "1"),
            ("pid_process_denom", "2"),
        ]);
        assert_eq!(extract_rpm_filter_rate(Some(&m)), 4000.0);
        let m = h(&[("looptime", "125"), ("pid_process_denom", "1")]);
        assert_eq!(extract_rpm_filter_rate(Some(&m)), 8000.0);
        // Missing or invalid values fall back to the default rate and a denominator of 1
        assert_eq!(
            extract_rpm_filter_rate(None),
            RPM_FILTER_DEFAULT_GYRO_RATE_HZ
        );
        let m = h(&[("looptime", "0"), ("pid_process_denom", "0")]);
        assert_eq!(
            extract_rpm_filter_rate(Some(&m)),
            RPM_FILTER_DEFAULT_GYRO_RATE_HZ
        );
    }

    #[test]
    fn test_generate_rpm_filter_curves_keeps_clamped_center_in_range() {
        // H3 raw 2100 Hz is above the 2000 Hz plot limit, but Betaflight clamps the center to
        // 0.48 * 4000 = 1920 Hz, which is inside the plot
        let cfg = test_rpm_config(3, vec![1.0, 1.0, 1.0]);
        let curves = generate_rpm_filter_curves(&cfg, 700.0, 2000.0, 4000, 4000.0);
        let h3 = curves.iter().find(|c| c.0 == 3).expect("H3 kept");
        assert_eq!(h3.3, 1920.0);
    }

    #[test]
    fn test_pt1_response() {
        // At cutoff frequency, magnitude should be ~0.707 (-3dB)
        let response = pt1_response(100.0, 100.0);
        assert!((response - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.001);

        // At 10x cutoff frequency, should be much lower
        let response_10x = pt1_response(1000.0, 100.0);
        assert!(response_10x < 0.1);
    }

    #[test]
    fn test_pt2_response() {
        // At cutoff, should be ~ -3 dB
        let r_fc = pt2_response(200.0, 200.0);
        assert!((r_fc - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.001);
        // Far above cutoff, very small
        let r_10x = pt2_response(2000.0, 200.0);
        assert!(r_10x < 0.02);
    }

    #[test]
    fn test_pt4_response() {
        // At cutoff, should be ~ -3 dB
        let r_fc = pt4_response(150.0, 150.0);
        assert!((r_fc - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.001);
    }

    #[test]
    fn test_pt3_response() {
        // At cutoff, should be ~ -3 dB
        let r_fc = pt3_response(300.0, 300.0);
        assert!((r_fc - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.001);
        // Far above cutoff, very small
        let r_10x = pt3_response(3000.0, 300.0);
        assert!(r_10x < 0.01);
    }

    #[test]
    fn test_extract_gyro_rate_with_sync_denom() {
        let headers = vec![
            ("looptime".to_string(), "125".to_string()), // 1e6 / 125 = 8000 Hz
            ("gyro_sync_denom".to_string(), "2".to_string()), // -> 4000 Hz
        ];
        let rate = extract_gyro_rate(Some(&headers));
        assert_eq!(rate, Some(4000.0));
    }

    #[test]
    fn test_filter_type_conversion() {
        assert_eq!(FilterType::from_u32(0), Some(FilterType::PT1));
        assert_eq!(FilterType::from_u32(1), Some(FilterType::Biquad));
        assert_eq!(FilterType::from_u32(2), Some(FilterType::PT2));
        assert_eq!(FilterType::from_u32(3), Some(FilterType::PT3));
        assert_eq!(FilterType::from_u32(99), None);
    }

    #[test]
    fn test_dynamic_cutoffs_parsing() {
        assert_eq!(parse_dynamic_cutoffs("75,150"), (75.0, 150.0));
        assert_eq!(parse_dynamic_cutoffs("\"100,200\""), (100.0, 200.0));
        assert_eq!(parse_dynamic_cutoffs("invalid"), (0.0, 0.0));
    }

    #[test]
    fn test_imuf_filter_parsing() {
        let headers = vec![
            ("IMUF_lowpass_roll".to_string(), "90".to_string()),
            ("IMUF_lowpass_pitch".to_string(), "95".to_string()),
            ("IMUF_lowpass_yaw".to_string(), "85".to_string()),
            ("IMUF_roll_q".to_string(), "8000".to_string()),
            ("IMUF_pitch_q".to_string(), "7500".to_string()),
            ("IMUF_yaw_q".to_string(), "9000".to_string()),
            ("IMUF_ptn_order".to_string(), "3".to_string()),
        ];

        let config = parse_imuf_filters_with_gyro_rate(&headers, None);

        // Check roll axis
        assert!(config.gyro[0].imuf.is_some());
        let roll_imuf = config.gyro[0].imuf.as_ref().unwrap();
        assert_eq!(roll_imuf.lowpass_cutoff_hz, 90.0);
        assert_eq!(roll_imuf.q_factor, 8.0); // 8000 / 1000
        assert_eq!(roll_imuf.ptn_order, 3);
        assert!(roll_imuf.enabled);

        // Check pitch axis
        assert!(config.gyro[1].imuf.is_some());
        let pitch_imuf = config.gyro[1].imuf.as_ref().unwrap();
        assert_eq!(pitch_imuf.lowpass_cutoff_hz, 95.0);
        assert_eq!(pitch_imuf.q_factor, 7.5); // 7500 / 1000

        // Check yaw axis
        assert!(config.gyro[2].imuf.is_some());
        let yaw_imuf = config.gyro[2].imuf.as_ref().unwrap();
        assert_eq!(yaw_imuf.lowpass_cutoff_hz, 85.0);
        assert_eq!(yaw_imuf.q_factor, 9.0); // 9000 / 1000
    }

    #[test]
    fn test_imuf_filter_detection() {
        let headers = vec![
            ("IMUF_lowpass_roll".to_string(), "90".to_string()),
            ("IMUF_ptn_order".to_string(), "2".to_string()),
        ];

        let config = parse_filter_config(&headers);
        assert!(config.gyro[0].imuf.is_some());
    }

    #[test]
    fn test_ptn_calculation_functions() {
        // Test PTN scaling factors
        assert_eq!(calculate_ptn_per_stage_cutoff(100.0, 1), 100.0); // PT1 no scaling
        assert!((calculate_ptn_per_stage_cutoff(100.0, 2) - 155.377).abs() < 0.1); // PT2 scaling
        assert!((calculate_ptn_per_stage_cutoff(100.0, 3) - 196.146).abs() < 0.1); // PT3 scaling
        assert!((calculate_ptn_per_stage_cutoff(100.0, 4) - 229.896).abs() < 0.1);
        // PT4 scaling
    }

    #[test]
    fn test_imuf_filter_curves() {
        let imuf_config = ImufFilterConfig {
            lowpass_cutoff_hz: 100.0,
            ptn_order: 2,
            q_factor: 8.0,
            revision: Some(256),
            pseudo_kalman_w: None,
            effective_cutoff_hz: 155.4, // Per-stage: 100 * 1.553773974
            enabled: true,
        };

        let axis_config = AxisFilterConfig {
            lpf1: None,
            lpf2: None,
            dynamic_lpf1: None,
            imuf: Some(imuf_config),
        };

        let curves = generate_individual_filter_curves(&axis_config, 1000.0, 100, true);
        assert!(!curves.is_empty()); // Should have at least combined curve, possibly per-stage too

        // Check combined response curve (red line - shows user-configured PT2 @ 100Hz)
        let (combined_label, _curve_points, combined_cutoff) = &curves[0];
        assert!(combined_label.contains("IMUF v256"));
        assert!(combined_label.contains("PT2")); // Should say "PT2" for user-configured filter
        assert!(combined_label.contains("100Hz"));
        assert!(!combined_label.contains("per-stage")); // Combined curve shouldn't have per-stage
        assert_eq!(*combined_cutoff, 100.0);

        // Check for per-stage curve if different enough (gray line - shows Two PT1 @ 155Hz)
        if curves.len() > 1 {
            let (stage_label, _, stage_cutoff) = &curves[1];
            assert!(stage_label.contains("IMUF v256"));
            assert!(stage_label.contains("Two PT1")); // Should say "Two PT1" for per-stage breakdown
            assert!(stage_label.contains("per-stage"));
            assert!(*stage_cutoff > 150.0); // Should be scaled up for Butterworth correction
        }
    }

    #[test]
    fn test_biquad_response_with_variable_q() {
        // Test Butterworth Q (0.707 = 1/sqrt(2))
        let butterworth_q = std::f64::consts::FRAC_1_SQRT_2;
        let response_butterworth = biquad_response_with_q(100.0, 100.0, butterworth_q);
        // At cutoff frequency with Butterworth Q, should be ~-3dB (0.707)
        assert!((response_butterworth - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.001);

        // Test with higher Q (sharper peak before cutoff)
        let response_high_q = biquad_response_with_q(80.0, 100.0, 2.0);
        let response_butterworth_80 = biquad_response_with_q(80.0, 100.0, butterworth_q);
        assert!(response_high_q > response_butterworth_80);

        // Test with lower Q (flatter response)
        let response_low_q = biquad_response_with_q(80.0, 100.0, 0.5);
        assert!(response_low_q < response_butterworth_80);

        // Test backward compatibility - biquad_response should use Butterworth
        let response_default = biquad_response(100.0, 100.0);
        assert!((response_default - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.001);
    }

    #[test]
    fn test_biquad_q_factor_parsing_betaflight() {
        let headers = vec![
            ("gyro_lpf1_type".to_string(), "1".to_string()), // BIQUAD
            ("gyro_lpf1_static_hz".to_string(), "100".to_string()),
            ("gyro_lowpass_q".to_string(), "2.0".to_string()),
            ("dterm_lpf1_type".to_string(), "1".to_string()),
            ("dterm_lpf1_static_hz".to_string(), "95".to_string()),
            ("dterm_lowpass_q".to_string(), "1.5".to_string()),
        ];

        let config = parse_betaflight_filters(&headers);

        // Check gyro LPF1 Q-factor
        assert!(config.gyro[0].lpf1.is_some());
        let gyro_lpf1 = config.gyro[0].lpf1.as_ref().unwrap();
        assert_eq!(gyro_lpf1.filter_type, FilterType::Biquad);
        assert_eq!(gyro_lpf1.cutoff_hz, 100.0);
        assert!(gyro_lpf1.q_factor.is_some());
        assert!((gyro_lpf1.q_factor.unwrap() - 2.0).abs() < 0.01);

        // Check D-term LPF1 Q-factor
        assert!(config.dterm[0].lpf1.is_some());
        let dterm_lpf1 = config.dterm[0].lpf1.as_ref().unwrap();
        assert_eq!(dterm_lpf1.filter_type, FilterType::Biquad);
        assert_eq!(dterm_lpf1.cutoff_hz, 95.0);
        assert!(dterm_lpf1.q_factor.is_some());
        assert!((dterm_lpf1.q_factor.unwrap() - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_biquad_q_factor_parsing_emuflight() {
        let headers = vec![
            ("gyro_lowpass_type".to_string(), "1".to_string()), // BIQUAD
            ("gyro_lowpass_hz_roll".to_string(), "100".to_string()),
            ("gyro_lowpass_q_roll".to_string(), "2.5".to_string()),
            ("gyro_lowpass_hz_pitch".to_string(), "105".to_string()),
            ("gyro_lowpass_q_pitch".to_string(), "2.3".to_string()),
        ];

        let config = parse_emuflight_filters(&headers);

        // Check roll axis
        assert!(config.gyro[0].lpf1.is_some());
        let roll_lpf1 = config.gyro[0].lpf1.as_ref().unwrap();
        assert_eq!(roll_lpf1.filter_type, FilterType::Biquad);
        assert_eq!(roll_lpf1.cutoff_hz, 100.0);
        assert!(roll_lpf1.q_factor.is_some());
        assert!((roll_lpf1.q_factor.unwrap() - 2.5).abs() < 0.01);

        // Check pitch axis
        assert!(config.gyro[1].lpf1.is_some());
        let pitch_lpf1 = config.gyro[1].lpf1.as_ref().unwrap();
        assert_eq!(pitch_lpf1.filter_type, FilterType::Biquad);
        assert_eq!(pitch_lpf1.cutoff_hz, 105.0);
        assert!(pitch_lpf1.q_factor.is_some());
        assert!((pitch_lpf1.q_factor.unwrap() - 2.3).abs() < 0.01);
    }

    #[test]
    fn test_biquad_curves_with_q_factor() {
        let lpf_biquad_high_q = FilterConfig {
            filter_type: FilterType::Biquad,
            cutoff_hz: 100.0,
            q_factor: Some(2.0),
            enabled: true,
        };

        let axis_config = AxisFilterConfig {
            lpf1: Some(lpf_biquad_high_q),
            lpf2: None,
            dynamic_lpf1: None,
            imuf: None,
        };

        let curves = generate_individual_filter_curves(&axis_config, 1000.0, 100, false);
        assert_eq!(curves.len(), 1);

        let (label, _curve_points, _cutoff) = &curves[0];
        assert!(label.contains("BIQUAD"));
        assert!(label.contains("Q=2.00"));
        assert!(label.contains("100Hz"));
    }

    #[test]
    fn test_imuf_pseudo_kalman_w_parsing() {
        let headers = vec![
            ("IMUF_lowpass_roll".to_string(), "90".to_string()),
            ("IMUF_ptn_order".to_string(), "2".to_string()),
            ("IMUF_w".to_string(), "32".to_string()),
        ];

        let config = parse_imuf_filters_with_gyro_rate(&headers, None);

        // Check that pseudo-Kalman window is parsed
        assert!(config.gyro[0].imuf.is_some());
        let imuf = config.gyro[0].imuf.as_ref().unwrap();
        assert_eq!(imuf.lowpass_cutoff_hz, 90.0);
        assert!(imuf.pseudo_kalman_w.is_some());
        assert_eq!(imuf.pseudo_kalman_w.unwrap(), 32.0);
    }

    #[test]
    fn test_ptn_filter_curves_generation() {
        // Test that PT2 generates both main and per-stage curves
        let lpf1_pt2 = FilterConfig {
            filter_type: FilterType::PT2,
            cutoff_hz: 100.0,
            q_factor: None,
            enabled: true,
        };

        let axis_config_pt2 = AxisFilterConfig {
            lpf1: Some(lpf1_pt2),
            lpf2: None,
            dynamic_lpf1: None,
            imuf: None,
        };

        let curves = generate_individual_filter_curves(&axis_config_pt2, 1000.0, 100, true);
        assert_eq!(
            curves.len(),
            2,
            "PT2 should generate 2 curves (main + per-stage)"
        );

        let (main_label, _, main_cutoff) = &curves[0];
        assert!(main_label.contains("LPF1"));
        assert!(main_label.contains("PT2"));
        assert!(main_label.contains("100Hz"));
        assert!(!main_label.contains("per-stage"));
        assert_eq!(*main_cutoff, 100.0);

        let (stage_label, _, stage_cutoff) = &curves[1];
        assert!(stage_label.contains("LPF1"));
        assert!(stage_label.contains("Two PT1"));
        assert!(stage_label.contains("per-stage"));
        assert!(*stage_cutoff > 150.0);

        // Test that PT1 generates only main curve
        let lpf1_pt1 = FilterConfig {
            filter_type: FilterType::PT1,
            cutoff_hz: 100.0,
            q_factor: None,
            enabled: true,
        };

        let axis_config_pt1 = AxisFilterConfig {
            lpf1: Some(lpf1_pt1),
            lpf2: None,
            dynamic_lpf1: None,
            imuf: None,
        };

        let curves_pt1 = generate_individual_filter_curves(&axis_config_pt1, 1000.0, 100, false);
        assert_eq!(
            curves_pt1.len(),
            1,
            "PT1 should generate only 1 curve (no per-stage)"
        );
        assert!(curves_pt1[0].0.contains("PT1"));
        assert!(!curves_pt1[0].0.contains("per-stage"));
    }

    #[test]
    fn test_biquad_curves_without_q_factor() {
        // Test BIQUAD without explicit Q-factor (should use Butterworth default)
        let lpf_biquad_default = FilterConfig {
            filter_type: FilterType::Biquad,
            cutoff_hz: 100.0,
            q_factor: None, // No explicit Q - should default to Butterworth
            enabled: true,
        };

        let axis_config = AxisFilterConfig {
            lpf1: Some(lpf_biquad_default),
            lpf2: None,
            dynamic_lpf1: None,
            imuf: None,
        };

        let curves = generate_individual_filter_curves(&axis_config, 1000.0, 100, false);
        assert_eq!(curves.len(), 1);

        let (_label, curve_points, _cutoff) = &curves[0];
        assert!(!curve_points.is_empty(), "Curve should have data points");

        // Find response closest to cutoff frequency
        let response_at_cutoff = curve_points
            .iter()
            .min_by(|(f1, _), (f2, _)| {
                let d1 = (*f1 - 100.0).abs();
                let d2 = (*f2 - 100.0).abs();
                d1.partial_cmp(&d2).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(_, response)| *response)
            .unwrap();

        // Should be ~-3dB (0.707) for Butterworth
        assert!(
            (response_at_cutoff - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.02,
            "BIQUAD without Q should default to Butterworth response (0.707 at cutoff), got {}",
            response_at_cutoff
        );
    }
}
