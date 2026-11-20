// src/data_analysis/calc_step_response.rs

use ndarray::{s, Array1, Array2};
use std::error::Error;

use crate::types::StepResponseResult;

use crate::constants::{
    APPLY_INDIVIDUAL_RESPONSE_Y_CORRECTION, ENABLE_NORMALIZED_STEADY_STATE_MEAN_CHECK,
    FRAME_LENGTH_S, HIGH_AUTHORITY_SETPOINT_THRESHOLD_DEG_S,
    LOW_AUTHORITY_SETPOINT_THRESHOLD_DEG_S, MOVEMENT_THRESHOLD_DEG_S,
    NORMALIZED_STEADY_STATE_MAX_VAL, NORMALIZED_STEADY_STATE_MEAN_MAX,
    NORMALIZED_STEADY_STATE_MEAN_MIN, NORMALIZED_STEADY_STATE_MIN_VAL, RESPONSE_LENGTH_S,
    STEADY_STATE_END_S, STEADY_STATE_START_S, SUPERPOSITION_FACTOR, TUKEY_ALPHA,
    WIENER_REGULARIZATION, Y_CORRECTION_MIN_UNNORMALIZED_MEAN_ABS,
};

/// Setpoint authority classification derived from the mean of per-window max setpoints.
/// Describes how aggressively the pilot flew during the step-response windows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetpointAuthority {
    Low,      // mean < LOW_AUTHORITY_SETPOINT_THRESHOLD_DEG_S
    Moderate, // LOW..HIGH
    High,     // >= HIGH_AUTHORITY_SETPOINT_THRESHOLD_DEG_S
}

impl SetpointAuthority {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Low => "LOW",
            Self::Moderate => "MODERATE",
            Self::High => "HIGH",
        }
    }

    pub fn is_low(&self) -> bool {
        matches!(self, Self::Low)
    }
}

/// Compute SetpointAuthority and mean window-max setpoint from QC-passed window data.
/// Returns None if the slice is empty.
pub fn compute_setpoint_authority(
    valid_window_max_setpoints: &[f32],
) -> Option<(SetpointAuthority, f32)> {
    if valid_window_max_setpoints.is_empty() {
        return None;
    }
    let mean =
        valid_window_max_setpoints.iter().sum::<f32>() / valid_window_max_setpoints.len() as f32;
    let level = if mean < LOW_AUTHORITY_SETPOINT_THRESHOLD_DEG_S {
        SetpointAuthority::Low
    } else if mean < HIGH_AUTHORITY_SETPOINT_THRESHOLD_DEG_S {
        SetpointAuthority::Moderate
    } else {
        SetpointAuthority::High
    };
    Some((level, mean))
}

use crate::data_analysis::fft_utils;

// tukeywin, winstacker_contiguous, wiener_deconvolution_window, cumulative_sum,
// average_responses
// are assumed to be the same as in the previous full file output provided.
// For brevity, only calculate_step_response is shown with changes.

/// Creates a Tukey window for signal tapering with configurable alpha parameter.
pub fn tukeywin(num: usize, alpha: f64) -> Array1<f32> {
    if alpha <= 0.0 {
        return Array1::ones(num);
    } else if alpha >= 1.0 {
        // Full Hanning window when alpha = 1.0
        let mut window = Array1::<f32>::zeros(num);
        for i in 0..num {
            window[i] = 0.5
                * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / (num as f64 - 1.0)).cos()) as f32;
        }
        return window;
    }
    let mut window = Array1::<f32>::ones(num);
    let alpha_half = alpha / 2.0;
    let n_alpha = (alpha_half * (num as f64 - 1.0)).floor() as usize;
    // Apply cosine tapering to both ends
    for i in 0..n_alpha {
        window[i] = 0.5 * (1.0 + (std::f64::consts::PI * i as f64 / (n_alpha as f64)).cos()) as f32;
        window[num - 1 - i] = window[i];
    }
    window
}

/// Generates overlapping windowed segments from input/output data for parallel analysis.
fn winstacker_contiguous(
    input_data: &Array1<f32>,
    output_data: &Array1<f32>,
    frame_length_samples: usize,
    superposition_factor: usize,
) -> (Array2<f32>, Array2<f32>) {
    let total_len = input_data.len();
    if total_len == 0 || frame_length_samples == 0 || superposition_factor == 0 {
        return (Array2::zeros((0, 0)), Array2::zeros((0, 0)));
    }
    let shift = frame_length_samples / superposition_factor;
    if shift == 0 {
        eprintln!("⚠️  Window shift is zero.");
        return (Array2::zeros((0, 0)), Array2::zeros((0, 0)));
    }
    let num_windows = if total_len >= frame_length_samples {
        (total_len - frame_length_samples) / shift + 1
    } else {
        0
    };
    if num_windows == 0 {
        return (Array2::zeros((0, 0)), Array2::zeros((0, 0)));
    }
    let mut stacked_input = Array2::<f32>::zeros((num_windows, frame_length_samples));
    let mut stacked_output = Array2::<f32>::zeros((num_windows, frame_length_samples));
    for i in 0..num_windows {
        let start = i * shift;
        let end = start + frame_length_samples;
        stacked_input
            .row_mut(i)
            .assign(&input_data.slice(s![start..end]));
        stacked_output
            .row_mut(i)
            .assign(&output_data.slice(s![start..end]));
    }
    (stacked_input, stacked_output)
}

/// Performs Wiener deconvolution to extract system impulse response from input/output signals.
fn wiener_deconvolution_window(
    input_window: &Array1<f32>,
    output_window: &Array1<f32>,
    _sample_rate: f64,
) -> Array1<f32> {
    let n = input_window.len();
    if n == 0 {
        return Array1::zeros(n);
    }
    let padded_n = n.next_power_of_two();
    let mut input_padded_vec = vec![0.0f32; padded_n];
    if n > 0 {
        input_padded_vec[0..n].copy_from_slice(input_window.as_slice().unwrap_or_default());
    }
    let input_padded = Array1::from(input_padded_vec);
    let mut output_padded_vec = vec![0.0f32; padded_n];
    if n > 0 {
        output_padded_vec[0..n].copy_from_slice(output_window.as_slice().unwrap_or_default());
    }
    let output_padded = Array1::from(output_padded_vec);
    let h_spec = fft_utils::fft_forward(&input_padded);
    let g_spec = fft_utils::fft_forward(&output_padded);
    if h_spec.is_empty() || g_spec.is_empty() || h_spec.len() != g_spec.len() {
        eprintln!("⚠️  FFT output empty/mismatch in Wiener deconvolution.");
        return Array1::zeros(n);
    }
    // Scale both spectra by 1/N before regularizing, as PIDtoolbox does (PTstepcalc.m: G = a/length(a)).
    // This makes the regularization term act as WIENER_REGULARIZATION * N^2 on unscaled FFTs.
    let regularization_term = WIENER_REGULARIZATION;
    let spectrum_scale = 1.0 / padded_n as f32;
    let epsilon = 1e-9;
    let mut deconvolved_spec = Array1::<num_complex::Complex32>::zeros(h_spec.len());
    for i in 0..h_spec.len() {
        let h = h_spec[i] * spectrum_scale;
        let g = g_spec[i] * spectrum_scale;
        let h_conj = h.conj();
        let denominator = (h * h_conj).re + regularization_term;
        if denominator.abs() > epsilon {
            deconvolved_spec[i] = (g * h_conj) / denominator;
        } else {
            deconvolved_spec[i] = num_complex::Complex32::new(0.0, 0.0);
        }
    }
    let deconvolved_impulse = fft_utils::fft_inverse(&deconvolved_spec, padded_n);
    if n > 0 {
        deconvolved_impulse.slice(s![0..n]).to_owned()
    } else {
        Array1::zeros(0)
    }
}

/// Converts impulse response to step response via cumulative integration.
fn cumulative_sum(data: &Array1<f32>) -> Array1<f32> {
    let mut cumulative = Array1::<f32>::zeros(data.len());
    let mut current_sum = 0.0;
    for (i, &val) in data.iter().enumerate() {
        if val.is_finite() {
            current_sum += val;
        } else {
            eprintln!("⚠️  Non-finite impulse value ({val}) at index {i}.");
        }
        cumulative[i] = current_sum;
    }
    cumulative
}

/// Computes weighted average of multiple step responses for final result.
pub fn average_responses(
    stacked_responses: &Array2<f32>,
    weights: &Array1<f32>,
    response_len: usize,
) -> Result<Array1<f64>, Box<dyn Error>> {
    let num_windows = stacked_responses.shape()[0];
    if num_windows == 0
        || response_len == 0
        || weights.len() != num_windows
        || stacked_responses.shape()[1] != response_len
    {
        return Err("Input data mismatch for average_responses".into());
    }
    let mut averaged_response = Array1::<f64>::zeros(response_len);
    let mut active_window_counts = Array1::<f64>::zeros(response_len);
    for i in 0..num_windows {
        let weight = weights[i] as f64;
        if weight <= 1e-9 {
            continue;
        }
        for j in 0..response_len {
            let response_value = stacked_responses[[i, j]] as f64;
            if response_value.is_finite() {
                averaged_response[j] += response_value * weight;
                active_window_counts[j] += weight;
            }
        }
    }
    for j in 0..response_len {
        if active_window_counts[j] > 1e-9 {
            averaged_response[j] /= active_window_counts[j];
        }
    }
    Ok(averaged_response)
}

/// Main function to calculate step response from setpoint and gyro data using windowed Wiener deconvolution.
/// Returns time array, stacked responses matrix, and max setpoints for each valid window.
pub fn calculate_step_response(
    _time: &Array1<f64>,
    setpoint: &Array1<f32>,
    gyro_filtered_input: &Array1<f32>,
    sample_rate: f64,
) -> Result<StepResponseResult, Box<dyn Error>> {
    if setpoint.is_empty()
        || gyro_filtered_input.is_empty()
        || setpoint.len() != gyro_filtered_input.len()
        || sample_rate <= 0.0
    {
        return Err("Invalid input to calculate_step_response".into());
    }

    let frame_length_samples = (FRAME_LENGTH_S * sample_rate).ceil() as usize;
    let response_length_samples = (RESPONSE_LENGTH_S * sample_rate).ceil() as usize;

    if frame_length_samples == 0 || response_length_samples == 0 {
        return Err("Calculated window length is zero.".into());
    }

    // Define steady-state region for quality control checks
    let ss_start_sample = (STEADY_STATE_START_S * sample_rate).floor() as usize;
    let ss_end_sample = (STEADY_STATE_END_S * sample_rate).ceil() as usize;
    let effective_ss_start_sample = ss_start_sample.min(response_length_samples.saturating_sub(1));
    let effective_ss_end_sample = ss_end_sample
        .min(response_length_samples)
        .max(effective_ss_start_sample + 1);

    let mut stacked_step_responses_qc: Vec<Array1<f32>> = Vec::new();
    let mut window_max_setpoints_qc: Vec<f32> = Vec::new();

    // Generate overlapping windows for parallel processing
    let (stacked_input_raw, stacked_output_raw) = winstacker_contiguous(
        setpoint,
        gyro_filtered_input,
        frame_length_samples,
        SUPERPOSITION_FACTOR,
    );

    let num_windows = stacked_input_raw.shape()[0];
    if num_windows == 0 {
        return Err("No complete windows generated.".into());
    }

    let window_func = tukeywin(frame_length_samples, TUKEY_ALPHA);

    // Process each window through deconvolution and quality control
    for i in 0..num_windows {
        let input_window_raw = stacked_input_raw.row(i).to_owned();
        let output_window_raw = stacked_output_raw.row(i).to_owned();

        // Filter out low-activity windows based on setpoint magnitude
        let max_setpoint_in_window = input_window_raw
            .iter()
            .fold(0.0f32, |max_val, &v| max_val.max(v.abs()));
        if max_setpoint_in_window < MOVEMENT_THRESHOLD_DEG_S as f32 {
            continue;
        }

        // Apply windowing function to reduce spectral leakage
        let input_window_windowed = &input_window_raw * &window_func;
        let output_window_windowed = &output_window_raw * &window_func;

        // Extract impulse response via Wiener deconvolution
        let impulse_response = wiener_deconvolution_window(
            &input_window_windowed,
            &output_window_windowed,
            sample_rate,
        );
        let impulse_response_truncated = if impulse_response.len() >= frame_length_samples {
            impulse_response
                .slice(s![0..frame_length_samples])
                .to_owned()
        } else {
            // eprintln!("Warning: Impulse response too short ({} vs {}) for window {}. Skipping.", impulse_response.len(), frame_length_samples, i);
            continue;
        };
        if impulse_response_truncated.is_empty() {
            continue;
        }

        // Convert impulse to step response and truncate to desired length
        let unnormalized_step_response = cumulative_sum(&impulse_response_truncated);
        if unnormalized_step_response.len() < response_length_samples {
            continue;
        }
        let truncated_unnormalized_response = unnormalized_step_response
            .slice(s![0..response_length_samples])
            .to_owned();

        let mut response_for_qc = truncated_unnormalized_response.clone();
        let mut y_correction_attempted_and_valid_for_qc = false;

        // Apply Y-correction (normalization) if enabled and conditions are met
        if APPLY_INDIVIDUAL_RESPONSE_Y_CORRECTION {
            if effective_ss_start_sample < effective_ss_end_sample
                && truncated_unnormalized_response.len() >= effective_ss_end_sample
            {
                let unnorm_ss_segment = truncated_unnormalized_response
                    .slice(s![effective_ss_start_sample..effective_ss_end_sample]);
                if let Some(unnorm_ss_mean) = unnorm_ss_segment.mean() {
                    if unnorm_ss_mean.is_finite()
                        && unnorm_ss_mean.abs() > Y_CORRECTION_MIN_UNNORMALIZED_MEAN_ABS
                    {
                        // ** MODIFIED: Standard Normalization for Y-Correction **
                        // Normalize response to target steady-state of 1.0
                        response_for_qc =
                            truncated_unnormalized_response.mapv(|v| v / unnorm_ss_mean);
                        y_correction_attempted_and_valid_for_qc = true;
                    } else {
                        // Unnormalized mean is too small or not finite, Y-correction cannot be reliably applied.
                        // Window will proceed to QC with its unnormalized response if APPLY_INDIVIDUAL_RESPONSE_Y_CORRECTION is false,
                        // or be effectively skipped for Y-corrected QC if APPLY_INDIVIDUAL_RESPONSE_Y_CORRECTION is true
                        // (as y_correction_attempted_and_valid_for_qc remains false).
                    }
                }
            }
        } else {
            // If Y-correction is not applied, QC will run on the unnormalized response.
            // The NORMALIZED_STEADY_STATE constants might not be appropriate in this case.
            // This path assumes QC constants are set considering unnormalized data if Y-correction is off.
            y_correction_attempted_and_valid_for_qc = true; // Allow QC to proceed on unnormalized data
        }

        // --- Quality Control ---
        // If Y-correction is enabled, QC should only run if Y-correction was successfully applied.
        // If Y-correction is disabled, QC runs on the unnormalized data.
        // Quality control: check steady-state values against thresholds
        let mut current_window_passes_qc = false;
        if (!APPLY_INDIVIDUAL_RESPONSE_Y_CORRECTION || y_correction_attempted_and_valid_for_qc)
            && effective_ss_start_sample < effective_ss_end_sample
            && response_for_qc.len() >= effective_ss_end_sample
        {
            let qc_ss_segment =
                response_for_qc.slice(s![effective_ss_start_sample..effective_ss_end_sample]);
            if !qc_ss_segment.is_empty() {
                let min_val_ss = qc_ss_segment.iter().fold(f32::INFINITY, |a, &b| a.min(b));
                let max_val_ss = qc_ss_segment
                    .iter()
                    .fold(f32::NEG_INFINITY, |a, &b| a.max(b));
                let mean_val_ss_opt = qc_ss_segment.mean();

                // Check min/max bounds for steady-state region
                if min_val_ss.is_finite()
                    && max_val_ss.is_finite()
                    && min_val_ss > NORMALIZED_STEADY_STATE_MIN_VAL
                    && max_val_ss < NORMALIZED_STEADY_STATE_MAX_VAL
                {
                    // Optional additional check on steady-state mean
                    if ENABLE_NORMALIZED_STEADY_STATE_MEAN_CHECK {
                        if let Some(mean_val_ss) = mean_val_ss_opt {
                            if mean_val_ss.is_finite()
                                && mean_val_ss > NORMALIZED_STEADY_STATE_MEAN_MIN
                                && mean_val_ss < NORMALIZED_STEADY_STATE_MEAN_MAX
                            {
                                current_window_passes_qc = true;
                            }
                        }
                    } else {
                        current_window_passes_qc = true;
                    }
                }
            }
        }

        // Collect windows that pass quality control
        if current_window_passes_qc {
            stacked_step_responses_qc.push(response_for_qc);
            window_max_setpoints_qc.push(max_setpoint_in_window);
        }
    }
    if stacked_step_responses_qc.is_empty() {
        return Err("No windows passed quality control.".into());
    }

    let rows = stacked_step_responses_qc.len();
    // Assuming all inner vectors should have response_length_samples.
    // If not, this needs to be the maximum length or handled differently.
    let cols = response_length_samples;

    // Convert validated responses to Array2 format for output
    let valid_stacked_responses = Array2::from_shape_fn((rows, cols), |(r, c)| {
        stacked_step_responses_qc
            .get(r)
            .and_then(|row| row.get(c))
            .copied()
            .unwrap_or(0.0)
    });
    let valid_window_max_setpoints = Array1::from(window_max_setpoints_qc);
    // Sample i is at i / sample_rate seconds (same time axis as PIDtoolbox: t = 0 : 1/lograte).
    let response_time = Array1::from_shape_fn(response_length_samples, |i| i as f64 / sample_rate);

    Ok((
        response_time,
        valid_stacked_responses,
        valid_window_max_setpoints,
    ))
}

/// Calculates Delay Time (Td) - the time for the step response to reach 50% of its final value.
/// Uses linear interpolation for a precise determination of the 50% threshold crossing.
/// A fixed offset of -1ms is applied to the calculated time (in milliseconds) before converting to seconds.
///
/// # Arguments
/// * `normalized_response` - A 1D array of the normalized step response (expected to go from ~0 to ~1).
/// * `sample_rate` - The sample rate of the response data in Hz.
///
/// # Returns
/// `Some(f64)` containing the delay time in seconds, or `None` if the response is empty,
/// sample rate is invalid, or the 50% threshold is not crossed.
pub fn calculate_delay_time(normalized_response: &Array1<f64>, sample_rate: f64) -> Option<f64> {
    if normalized_response.is_empty() || sample_rate <= 0.0 {
        return None;
    }

    // Validate that this looks like a normalized step response
    let final_value = normalized_response[normalized_response.len() - 1];
    if !(0.1..=2.0).contains(&final_value) {
        // Response doesn't appear to be a normalized step response
        return None;
    }

    let threshold = 0.5;
    let mut interpolated_idx_0_based: Option<f64> = None;

    // Check if the first point is at or above the threshold
    if normalized_response[0] >= threshold {
        interpolated_idx_0_based = Some(0.0);
    } else {
        for idx in 1..normalized_response.len() {
            let y0 = normalized_response[idx - 1];
            let y1 = normalized_response[idx];

            if y0 < threshold && y1 >= threshold {
                // Found a crossing
                let x0 = (idx - 1) as f64; // 0-based index of point before crossing
                let x1 = idx as f64; // 0-based index of point at or after crossing

                if (y1 - y0).abs() < 1e-9 {
                    // Avoid division by zero if y1 is very close to y0
                    // This implies the segment is flat and at/crossing the threshold.
                    // Take the first point of this segment that is >= threshold.
                    interpolated_idx_0_based = Some(x1);
                } else {
                    // Linear interpolation: x = x0 + (threshold - y0) * (x1 - x0) / (y1 - y0)
                    let t = (threshold - y0) / (y1 - y0);
                    interpolated_idx_0_based = Some(x0 + t * (x1 - x0));
                }
                break; // Found the first crossing
            }
        }
    }

    if let Some(idx_val) = interpolated_idx_0_based {
        // Calculation of delay time in seconds:
        // 1. Convert the 0-based (potentially fractional) interpolated index to a 1-based sample count.
        //    This represents the number of samples (or fractional samples) into the response where the threshold is crossed.
        let time_samples_1_based = idx_val + 1.0;

        // 2. Determine sample rate in samples per millisecond for time conversion.
        let sample_rate_per_ms = sample_rate / 1000.0;
        if sample_rate_per_ms <= 1e-9 {
            // Avoid division by zero or issues with very low sample rates
            return None;
        }

        // 3. Calculate the time to threshold in milliseconds.
        let time_ms = time_samples_1_based / sample_rate_per_ms;

        // 4. Apply a 1ms subtraction offset to the time in milliseconds.
        let delay_ms = time_ms - 1.0;

        // 5. Convert to seconds and ensure the result is non-negative.
        Some((delay_ms / 1000.0).max(0.0))
    } else {
        None // Threshold not crossed
    }
}

/// Finds the peak (maximum) value in a 1D response data array.
/// Returns None if the array is empty or contains no finite values.
pub fn find_peak_value(response_data: &Array1<f64>) -> Option<f64> {
    if response_data.is_empty() {
        return None;
    }
    let mut peak_val = f64::NEG_INFINITY;
    let mut found_finite = false;
    for &val in response_data.iter() {
        if val.is_finite() {
            if !found_finite || val > peak_val {
                peak_val = val;
            }
            found_finite = true;
        }
    }
    if found_finite {
        Some(peak_val)
    } else {
        None
    }
}

// src/data_analysis/calc_step_response.rs

#[cfg(test)]
mod tests {
    use super::*;

    // 1001 Hz makes RESPONSE_LENGTH_S * rate non-integral (500.5), so the length rounds up.
    const TEST_SAMPLE_RATES: [f64; 2] = [1000.0, 1001.0];

    #[test]
    fn response_time_axis_is_sample_index_over_rate() {
        for rate in TEST_SAMPLE_RATES {
            // Identity plant: gyro follows setpoint, so at least one window passes QC.
            let n = (rate * 6.0) as usize;
            let mut x = 0.0f32;
            let setpoint: Array1<f32> = Array1::from_shape_fn(n, |i| {
                if i % 400 == 0 {
                    x = if x > 0.0 { -150.0 } else { 150.0 };
                }
                x
            });
            let time = Array1::from_shape_fn(n, |i| i as f64 / rate);
            let (t, resp, _) = calculate_step_response(&time, &setpoint, &setpoint, rate).unwrap();
            assert!(resp.shape()[0] > 0);
            assert_eq!(t.len(), (RESPONSE_LENGTH_S * rate).ceil() as usize);
            assert_eq!(t[0], 0.0);
            for i in 0..t.len() {
                assert!((t[i] - i as f64 / rate).abs() < 1e-12);
            }
        }
    }

    /// Mean of QC-passed step responses.
    fn mean_response(setpoint: &Array1<f32>, gyro: &Array1<f32>, rate: f64) -> Array1<f64> {
        let time = Array1::from_shape_fn(setpoint.len(), |i| i as f64 / rate);
        let (_, resp, _) = calculate_step_response(&time, setpoint, gyro, rate).unwrap();
        average_responses(&resp, &Array1::ones(resp.shape()[0]), resp.shape()[1]).unwrap()
    }

    /// 50% crossing, in samples from the first sample, with linear interpolation.
    fn crossing_50(r: &Array1<f64>) -> f64 {
        let i = r.iter().position(|&v| v >= 0.5).unwrap();
        if i == 0 {
            return 0.0;
        }
        (i - 1) as f64 + (0.5 - r[i - 1]) / (r[i] - r[i - 1])
    }

    #[test]
    fn identity_plant_gives_unit_step_with_no_delay() {
        let rate = 1000.0;
        let n = (rate * 8.0) as usize;
        let mut x = 0.0f32;
        let sp = Array1::from_shape_fn(n, |i| {
            if i % 400 == 0 {
                x = if x > 0.0 { -150.0 } else { 150.0 };
            }
            x
        });
        let r = mean_response(&sp, &sp, rate);
        // Steady state is normalized to 1.0; a plant with no lag crosses 50% within one sample.
        assert!((r[r.len() - 1] - 1.0).abs() < 0.1);
        assert!(crossing_50(&r) < 2.0);
    }

    #[test]
    fn first_order_lag_crossing_matches_plant() {
        // y[k] = y[k-1] + a (x[k-1] - y[k-1]), a = 1 / (1 + tau).
        // Step response s[k] = 1 - (1 - a)^k, so s = 0.5 at k = ln(0.5) / ln(1 - a).
        let rate = 1000.0;
        let tau = 8.0f32;
        let a = 1.0 / (1.0 + tau);
        let n = (rate * 12.0) as usize;
        let mut state = 12345u64;
        let mut level = 0.0f32;
        let sp = Array1::from_shape_fn(n, |i| {
            if i % 250 == 0 {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                level = ((state >> 33) as f32 / (1u64 << 31) as f32 - 0.5) * 600.0;
            }
            level
        });
        let mut y = 0.0f32;
        let gy = Array1::from_shape_fn(n, |i| {
            if i > 0 {
                y += a * (sp[i - 1] - y);
            }
            y
        });
        let r = mean_response(&sp, &gy, rate);
        let expected = (0.5f64).ln() / (1.0 - a as f64).ln();
        // Regularization low-passes the estimate, so allow 1.5 samples.
        assert!(
            (crossing_50(&r) - expected).abs() < 1.5,
            "crossing {} expected {}",
            crossing_50(&r),
            expected
        );
    }
}
