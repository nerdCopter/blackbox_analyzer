// src/plot_functions/plot_filter_group_delay.rs

use std::error::Error;

use crate::constants::{
    COLOR_GROUP_DELAY_DTERM, COLOR_GROUP_DELAY_DTERM_MAX_CUTOFF, COLOR_GROUP_DELAY_GYRO,
    COLOR_GROUP_DELAY_GYRO_MAX_CUTOFF, GROUP_DELAY_PLOT_MAX_HZ, LINE_WIDTH_PLOT,
    UNIFIED_Y_AXIS_HEADROOM_SCALE,
};
use crate::data_analysis::filter_group_delay::{
    delays_text, group_delay_curve_ms, AxisGroupDelay, ChainDelay,
};
use crate::plot_framework::{draw_stacked_plot, PlotSeries};

/// Series for one chain: the chain at the (minimum) cutoff, plus the maximum-cutoff variant when
/// LPF1 is dynamic. Legend text starts with the signal; the delay values come from `delays_text`.
fn chain_series(
    chain: &ChainDelay,
    color: plotters::style::RGBColor,
    color_max: plotters::style::RGBColor,
) -> Vec<PlotSeries> {
    let mut series = vec![PlotSeries {
        data: group_delay_curve_ms(&chain.stages),
        label: format!("{} chain: {}", chain.signal, delays_text(chain)),
        color,
        stroke_width: LINE_WIDTH_PLOT,
    }];
    if let Some(ref fast) = chain.stages_at_max_cutoff {
        series.push(PlotSeries {
            data: group_delay_curve_ms(fast),
            label: format!("{} chain at max dynamic cutoff", chain.signal),
            color: color_max,
            stroke_width: LINE_WIDTH_PLOT,
        });
    }
    series
}

/// Generates the stacked per-axis theoretical filter group delay plot (gyro and D-term chains).
/// Delay is computed from the header filter settings, not measured from flight data.
pub fn plot_filter_group_delay(
    results: &[AxisGroupDelay],
    root_name: &str,
) -> Result<(), Box<dyn Error>> {
    let output_file = format!("{root_name}_Filter_Group_Delay_stacked.png");
    let plot_type_name = "Filter Group Delay";

    // (chart title, y max, series) per axis; owned so the draw closure can be 'static.
    let axis_plots: Vec<Option<(String, f64, Vec<PlotSeries>)>> = results
        .iter()
        .map(|r| {
            let mut series = Vec::new();
            if let Some(ref chain) = r.gyro {
                series.extend(chain_series(
                    chain,
                    *COLOR_GROUP_DELAY_GYRO,
                    *COLOR_GROUP_DELAY_GYRO_MAX_CUTOFF,
                ));
            }
            if let Some(ref chain) = r.dterm {
                series.extend(chain_series(
                    chain,
                    *COLOR_GROUP_DELAY_DTERM,
                    *COLOR_GROUP_DELAY_DTERM_MAX_CUTOFF,
                ));
            }
            if series.is_empty() {
                return None;
            }
            // Scale to the largest plotted delay (a high-Q biquad can peak above its DC value).
            let y_max = series
                .iter()
                .flat_map(|s| s.data.iter().map(|&(_, d)| d))
                .fold(0.0_f64, f64::max)
                * UNIFIED_Y_AXIS_HEADROOM_SCALE;
            Some((format!("{} Filter Group Delay", r.axis_name), y_max, series))
        })
        .collect();

    draw_stacked_plot(&output_file, root_name, plot_type_name, move |axis_index| {
        let (title, y_max, series) = axis_plots.get(axis_index)?.as_ref()?;
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
            0.0..GROUP_DELAY_PLOT_MAX_HZ,
            0.0..*y_max,
            series,
            "Frequency (Hz)".to_string(),
            "Group Delay (ms)".to_string(),
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // No filter chain on any axis: nothing to plot, so no PNG is created and the call succeeds.
    #[test]
    fn no_chain_creates_no_png() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        let results: Vec<AxisGroupDelay> = ["Roll", "Pitch", "Yaw"]
            .iter()
            .map(|n| AxisGroupDelay {
                axis_name: n.to_string(),
                gyro: None,
                dterm: None,
            })
            .collect();
        plot_filter_group_delay(&results, &root).expect("ok");
        let png = format!("{root}_Filter_Group_Delay_stacked.png");
        assert!(!std::path::Path::new(&png).exists());
    }
}
