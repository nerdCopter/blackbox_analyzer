// src/plot_functions/plot_setpoint_vs_gyro.rs

use plotters::style::RGBColor;
use std::error::Error;

use crate::axis_names::AXIS_NAMES;
use crate::data_analysis::feedforward::{axis_has_feedforward, ff_equivalent_scale};
use crate::data_input::pid_metadata::PidMetadata;
use crate::types::AllAxisPlotData2;

use crate::constants::{
    COLOR_SETPOINT_VS_GYRO_FF, COLOR_SETPOINT_VS_GYRO_GYRO, COLOR_SETPOINT_VS_GYRO_SP,
    LINE_WIDTH_PLOT, UNIFIED_Y_AXIS_HEADROOM_SCALE, UNIFIED_Y_AXIS_MIN_SCALE,
    UNIFIED_Y_AXIS_PERCENTILE,
};
use crate::data_analysis::filter_delay;
use crate::data_analysis::filter_delay::DelayAnalysisResult;
use crate::data_input::log_data::LogRowData;
use crate::plot_framework::{draw_stacked_plot, PlotSeries};

/// Generates the Stacked Setpoint vs Gyro Plot (Orange, Blue). An axis whose log carries
/// non-zero axisF on Betaflight also gets the P-equivalent feedforward trace (Green), deg/s.
pub fn plot_setpoint_vs_gyro(
    log_data: &[LogRowData],
    root_name: &str,
    sample_rate: Option<f64>,
    pid_metadata: &PidMetadata,
) -> Result<(), Box<dyn Error>> {
    let output_file_setpoint_gyro = format!("{root_name}_SetpointVsGyro_stacked.png");
    let plot_type_name = "Setpoint/Gyro";

    // Calculate filtering delay using enhanced cross-correlation
    let delay_analysis = if let Some(sr) = sample_rate {
        filter_delay::calculate_average_filtering_delay_comparison(log_data, sr)
    } else {
        DelayAnalysisResult {
            average_delay: None,
            results: Vec::new(),
            axis_delays: vec![None; AXIS_NAMES.len()],
        }
    };

    let delay_comparison_results = if !delay_analysis.results.is_empty() {
        Some(delay_analysis.results)
    } else {
        None
    };

    let mut axis_plot_data: AllAxisPlotData2 = Default::default();
    for row in log_data {
        if let Some(time) = row.time_sec {
            #[allow(clippy::needless_range_loop)]
            for axis_index in 0..AXIS_NAMES.len() {
                axis_plot_data[axis_index].push((
                    time,
                    row.setpoint[axis_index],
                    row.gyro[axis_index],
                ));
            }
        }
    }

    // Per axis: deg/s per axisF unit, only where the axis has feedforward to show.
    let ff_scales: Vec<Option<f64>> = (0..AXIS_NAMES.len())
        .map(|axis_index| {
            ff_equivalent_scale(pid_metadata, axis_index)
                .filter(|_| axis_has_feedforward(log_data, axis_index))
        })
        .collect();
    let ff_series_by_axis: Vec<Vec<(f64, f64)>> = (0..AXIS_NAMES.len())
        .map(|axis_index| match ff_scales[axis_index] {
            Some(scale) => log_data
                .iter()
                .filter_map(|row| Some((row.time_sec?, row.f_term[axis_index]? * scale)))
                .collect(),
            None => Vec::new(),
        })
        .collect();

    let color_sp: RGBColor = *COLOR_SETPOINT_VS_GYRO_SP;
    let color_gyro: RGBColor = *COLOR_SETPOINT_VS_GYRO_GYRO;
    let line_stroke_plot = LINE_WIDTH_PLOT;

    let half_range = unified_half_range(&axis_plot_data, &ff_series_by_axis);

    draw_stacked_plot(
        &output_file_setpoint_gyro,
        root_name,
        plot_type_name,
        move |axis_index| {
            let data = &axis_plot_data[axis_index];
            if data.is_empty() {
                return None;
            }

            let mut setpoint_series_data: Vec<(f64, f64)> = Vec::new();
            let mut gyro_series_data: Vec<(f64, f64)> = Vec::new();

            let mut time_min = f64::INFINITY;
            let mut time_max = f64::NEG_INFINITY;

            for (time, setpoint, gyro_filt) in data {
                time_min = time_min.min(*time);
                time_max = time_max.max(*time);

                if let Some(s) = setpoint {
                    setpoint_series_data.push((*time, *s));
                }
                if let Some(g) = gyro_filt {
                    gyro_series_data.push((*time, *g));
                }
            }

            if setpoint_series_data.is_empty() && gyro_series_data.is_empty() {
                return None;
            }

            // Use unified symmetric Y-axis range across all axes
            let x_range = time_min..time_max;
            let y_range = -half_range..half_range;

            let mut series = Vec::new();
            if !gyro_series_data.is_empty() {
                let gyro_label = if let Some(ref results) = delay_comparison_results {
                    // Show comparison of both methods if available - NO AVERAGING
                    let mut method_strings = Vec::new();
                    for result in results.iter() {
                        if let Some(freq) = result.frequency_hz {
                            method_strings.push(format!(
                                "{}: {:.1}ms@{:.0}Hz(c:{:.0}%)",
                                match result.method.as_str() {
                                    "Enhanced Cross-Correlation" => "Delay",
                                    _ => "Unknown",
                                },
                                result.delay_ms,
                                freq,
                                result.confidence * 100.0
                            ));
                        } else {
                            method_strings.push(format!(
                                "{}: {:.1}ms(c:{:.0}%)",
                                match result.method.as_str() {
                                    "Enhanced Cross-Correlation" => "Delay",
                                    _ => "Unknown",
                                },
                                result.delay_ms,
                                result.confidence * 100.0
                            ));
                        }
                    }
                    if method_strings.is_empty() {
                        "Gyro (gyroADC)".to_string()
                    } else {
                        format!("Gyro (gyroADC) - {}", method_strings.join(" vs "))
                    }
                } else {
                    "Gyro (gyroADC)".to_string()
                };
                series.push(PlotSeries {
                    data: gyro_series_data,
                    label: gyro_label,
                    color: color_gyro,
                    stroke_width: line_stroke_plot,
                });
            }
            if !setpoint_series_data.is_empty() {
                series.push(PlotSeries {
                    data: setpoint_series_data,
                    label: "Setpoint".to_string(),
                    color: color_sp,
                    stroke_width: line_stroke_plot,
                });
            }

            if !ff_series_by_axis[axis_index].is_empty() {
                series.push(PlotSeries {
                    data: ff_series_by_axis[axis_index].clone(),
                    label: "Feedforward (axisF, nominal P-equivalent)".to_string(),
                    color: *COLOR_SETPOINT_VS_GYRO_FF,
                    stroke_width: line_stroke_plot,
                });
            }

            Some((
                { format!("{} Setpoint vs Gyro", AXIS_NAMES[axis_index]) },
                x_range,
                y_range,
                series,
                "Time (s)".to_string(),
                "Value".to_string(),
            ))
        },
    )
}

/// Shared symmetric Y half-range: percentile of the absolute setpoint, gyro and
/// feedforward values across all axes, so every plotted trace shares one scale.
fn unified_half_range(
    axis_plot_data: &AllAxisPlotData2,
    ff_series_by_axis: &[Vec<(f64, f64)>],
) -> f64 {
    let mut all_abs_vals = Vec::new();
    for data in axis_plot_data.iter() {
        for (_, setpoint, gyro_filt) in data {
            all_abs_vals.extend(setpoint.iter().map(|s| s.abs()));
            all_abs_vals.extend(gyro_filt.iter().map(|g| g.abs()));
        }
    }
    for series in ff_series_by_axis {
        all_abs_vals.extend(series.iter().map(|(_, ff)| ff.abs()));
    }

    // The percentile keeps outliers (crashes, hard landings) from compressing normal flight.
    if all_abs_vals.is_empty() {
        return UNIFIED_Y_AXIS_MIN_SCALE;
    }
    all_abs_vals.sort_by(|a, b| a.total_cmp(b));
    let p95_idx = ((all_abs_vals.len() - 1) as f64 * UNIFIED_Y_AXIS_PERCENTILE).floor() as usize;
    (all_abs_vals[p95_idx] * UNIFIED_Y_AXIS_HEADROOM_SCALE).max(UNIFIED_Y_AXIS_MIN_SCALE)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plot_data(setpoint: f64, gyro: f64, n: usize) -> AllAxisPlotData2 {
        let mut data: AllAxisPlotData2 = Default::default();
        for axis in data.iter_mut() {
            for i in 0..n {
                axis.push((i as f64, Some(setpoint), Some(gyro)));
            }
        }
        data
    }

    #[test]
    fn half_range_without_feedforward_follows_setpoint_and_gyro() {
        let data = plot_data(-500.0, 400.0, 100);
        let empty: Vec<Vec<(f64, f64)>> = vec![Vec::new(); AXIS_NAMES.len()];
        let expected = 500.0 * UNIFIED_Y_AXIS_HEADROOM_SCALE;
        assert_eq!(unified_half_range(&data, &empty), expected);
    }

    #[test]
    fn half_range_covers_feedforward_above_setpoint_and_gyro() {
        let data = plot_data(300.0, 300.0, 100);
        let ff: Vec<Vec<(f64, f64)>> = (0..AXIS_NAMES.len())
            .map(|_| (0..100).map(|i| (i as f64, -1500.0)).collect())
            .collect();
        let half = unified_half_range(&data, &ff);
        assert!(
            half >= 1500.0,
            "feedforward trace must lie inside the range"
        );
    }
}

// src/plot_functions/plot_setpoint_vs_gyro.rs
