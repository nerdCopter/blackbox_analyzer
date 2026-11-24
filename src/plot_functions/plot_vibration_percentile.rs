// src/plot_functions/plot_vibration_percentile.rs

use std::error::Error;

use crate::constants::{
    COLOR_VIBRATION_PERCENTILE_FILT, COLOR_VIBRATION_PERCENTILE_UNFILT, LINE_WIDTH_PLOT,
    UNIFIED_Y_AXIS_HEADROOM_SCALE, VIBRATION_X_AXIS_MIN_SCALE, VIBRATION_X_AXIS_PERCENTILE_INDEX,
};
use crate::data_analysis::vibration_percentile::{
    percentile_summary_text, series_ecdf_points, VibrationPercentileResult, VibrationSeries,
    SIGNAL_FILTERED, SIGNAL_UNFILTERED,
};
use crate::plot_framework::{draw_stacked_plot, PlotSeries};

/// Legend text: signal name plus the shared percentile summary (same text as the console).
pub fn series_legend(signal: &str, series: &VibrationSeries) -> String {
    format!("{signal}: {}", percentile_summary_text(series))
}

/// Generates the stacked per-axis |gyro| cumulative distribution plot (filtered vs unfiltered).
/// The X-axis ends at the larger P99 of the two series times `UNIFIED_Y_AXIS_HEADROOM_SCALE`,
/// so a rare spike does not compress the bulk of the distribution.
pub fn plot_vibration_percentile(
    results: &[VibrationPercentileResult],
    root_name: &str,
    using_debug_fallback: bool,
    debug_mode_name: Option<&str>,
) -> Result<(), Box<dyn Error>> {
    let output_file = format!("{root_name}_Gyro_Vibration_Percentile_stacked.png");
    let plot_type_name = "Gyro Vibration Percentile";

    let unfilt_signal = super::format_debug_suffix(
        &format!("{SIGNAL_UNFILTERED} Gyro (gyroUnfilt)"),
        using_debug_fallback,
        debug_mode_name,
    );

    let filt_signal = format!("{SIGNAL_FILTERED} Gyro (gyroADC)");

    // (chart title, x max, series) per axis; owned so the draw closure can be 'static.
    let axis_plots: Vec<Option<(String, f64, Vec<PlotSeries>)>> = results
        .iter()
        .map(|r| {
            let mut series = Vec::new();
            let mut x_max: f64 = 0.0;
            // Unfiltered is drawn first, so the filtered curve stays on top.
            for (label, s) in r.present_series().into_iter().rev() {
                let (signal, color) = if label == SIGNAL_UNFILTERED {
                    (unfilt_signal.as_str(), *COLOR_VIBRATION_PERCENTILE_UNFILT)
                } else {
                    (filt_signal.as_str(), *COLOR_VIBRATION_PERCENTILE_FILT)
                };
                x_max = x_max.max(s.percentiles[VIBRATION_X_AXIS_PERCENTILE_INDEX]);
                series.push(PlotSeries {
                    data: series_ecdf_points(s),
                    label: series_legend(signal, s),
                    color,
                    stroke_width: LINE_WIDTH_PLOT,
                });
            }
            if series.is_empty() {
                return None;
            }
            let x_max = (x_max * UNIFIED_Y_AXIS_HEADROOM_SCALE).max(VIBRATION_X_AXIS_MIN_SCALE);
            Some((
                format!("{} |Gyro| Cumulative Distribution", r.axis_name),
                x_max,
                series,
            ))
        })
        .collect();

    draw_stacked_plot(&output_file, root_name, plot_type_name, move |axis_index| {
        let (title, x_max, series) = axis_plots.get(axis_index)?.as_ref()?;
        let series = series
            .iter()
            .map(|s| PlotSeries {
                data: s.data.clone(),
                label: s.label.clone(),
                color: s.color,
                stroke_width: s.stroke_width,
            })
            .collect();
        Some((
            title.clone(),
            0.0..*x_max,
            0.0..1.0,
            series,
            "|Gyro| (deg/s)".to_string(),
            "Cumulative Fraction of Samples".to_string(),
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // All axes empty: nothing to plot, so no PNG is created and the call succeeds.
    #[test]
    fn all_missing_input_creates_no_png() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        let results: Vec<VibrationPercentileResult> = ["Roll", "Pitch", "Yaw"]
            .iter()
            .map(|n| VibrationPercentileResult {
                axis_name: n.to_string(),
                filtered: None,
                unfiltered: None,
            })
            .collect();
        plot_vibration_percentile(&results, &root, false, None).expect("ok");
        let png = format!("{root}_Gyro_Vibration_Percentile_stacked.png");
        assert!(!std::path::Path::new(&png).exists());
    }
}
