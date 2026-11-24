// src/plot_functions/plot_coherence.rs

use std::error::Error;

use crate::constants::COHERENCE_BIN_MATCH_TOLERANCE;
use crate::constants::{
    COHERENCE_HIGH_THRESHOLD, COHERENCE_MEDIUM_THRESHOLD, COHERENCE_PLOT_MAX_HZ, COLOR_COHERENCE,
    COLOR_COHERENCE_HIGH_LINE, COLOR_COHERENCE_MEDIUM_LINE, LINE_WIDTH_PLOT,
};
use crate::data_analysis::coherence_analysis::{summary_text, CoherenceResult};
use crate::plot_framework::{draw_stacked_plot, PlotSeries};

/// Stroke width of the threshold reference lines.
const REFERENCE_LINE_WIDTH: u32 = 1;

/// Per-axis series for one result: coherence curve and the two threshold reference lines,
/// plus the X-axis upper limit. `None` when the result holds no bin.
fn axis_series(r: &CoherenceResult) -> Option<(f64, Vec<PlotSeries>)> {
    let last = *r.frequency_hz.last()?;
    let x_max = last.min(COHERENCE_PLOT_MAX_HZ);
    // Runs of adjacent bins; a gap of omitted bins ends a run so no line crosses it.
    let tolerance = r.bin_hz * COHERENCE_BIN_MATCH_TOLERANCE;
    let mut runs: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut previous: Option<f64> = None;
    for (&f, &c) in r.frequency_hz.iter().zip(&r.coherence) {
        if f > x_max {
            break;
        }
        if previous.is_none_or(|p| f - p > r.bin_hz + tolerance) {
            runs.push(Vec::new());
        }
        runs.last_mut()?.push((f, c));
        previous = Some(f);
    }
    // Every bin was beyond the plotted range: no measurement to draw.
    if runs.is_empty() {
        return None;
    }
    let line = |level: f64| vec![(0.0, level), (x_max, level)];
    // Only the first run carries the legend entry.
    let mut series: Vec<PlotSeries> = runs
        .into_iter()
        .enumerate()
        .map(|(i, data)| PlotSeries {
            data,
            label: if i == 0 {
                format!("Coherence ({})", summary_text(r))
            } else {
                String::new()
            },
            color: *COLOR_COHERENCE,
            stroke_width: LINE_WIDTH_PLOT,
        })
        .collect();
    series.push(PlotSeries {
        data: line(COHERENCE_HIGH_THRESHOLD),
        label: format!("High ({COHERENCE_HIGH_THRESHOLD:.1})"),
        color: *COLOR_COHERENCE_HIGH_LINE,
        stroke_width: REFERENCE_LINE_WIDTH,
    });
    series.push(PlotSeries {
        data: line(COHERENCE_MEDIUM_THRESHOLD),
        label: format!("Medium ({COHERENCE_MEDIUM_THRESHOLD:.1})"),
        color: *COLOR_COHERENCE_MEDIUM_LINE,
        stroke_width: REFERENCE_LINE_WIDTH,
    });
    Some((x_max, series))
}

/// Generates the stacked per-axis setpoint-gyro coherence plot. No file is written when no
/// axis has a result.
pub fn plot_coherence(results: &[CoherenceResult], root_name: &str) -> Result<(), Box<dyn Error>> {
    let output_file = format!("{root_name}_Coherence_stacked.png");
    let plot_type_name = "Setpoint-Gyro Coherence";

    // (chart title, x max, series) per axis name; owned so the draw closure can be 'static.
    let axis_plots: Vec<Option<(String, f64, Vec<PlotSeries>)>> = crate::axis_names::AXIS_NAMES
        .iter()
        .map(|name| {
            let r = results.iter().find(|r| r.axis_name == *name)?;
            let (x_max, series) = axis_series(r)?;
            Some((format!("{name} Setpoint-Gyro Coherence"), x_max, series))
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
            "Frequency (Hz)".to_string(),
            "Coherence".to_string(),
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_analysis::coherence_analysis::BandMean;

    fn result(axis: &str, freq: Vec<f64>, coh: Vec<f64>) -> CoherenceResult {
        CoherenceResult {
            axis_name: axis.to_string(),
            frequency_hz: freq,
            coherence: coh,
            segment_count: 20,
            bin_hz: 2.0,
            mean_coherence: Some(BandMean {
                mean: 0.5,
                bins: 49,
                bins_total: 50,
            }),
            coherent_bandwidth_hz: Some(10.0),
        }
    }

    // No axis has a result: nothing to plot, so no PNG is created and the call succeeds.
    #[test]
    fn no_results_creates_no_png() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        plot_coherence(&[], &root).expect("ok");
        assert!(!std::path::Path::new(&format!("{root}_Coherence_stacked.png")).exists());
    }

    // Spec: the curve stops at COHERENCE_PLOT_MAX_HZ and the axis limit follows it; the reference
    // lines sit at the shared thresholds and span the plotted range.
    #[test]
    fn series_cap_the_axis_and_place_reference_lines() {
        let freq: Vec<f64> = (1..=10).map(|i| f64::from(i) * 100.0).collect();
        let mut r = result("Roll", freq, vec![0.5; 10]);
        r.bin_hz = 100.0;
        let (x_max, series) = axis_series(&r).expect("series");
        assert_eq!(x_max, COHERENCE_PLOT_MAX_HZ);
        assert_eq!(series[0].data.len(), 5);
        assert!(series[0]
            .data
            .iter()
            .all(|&(f, _)| f <= COHERENCE_PLOT_MAX_HZ));
        assert_eq!(series[1].data, [(0.0, 0.7), (x_max, 0.7)]);
        assert_eq!(series[2].data, [(0.0, 0.4), (x_max, 0.4)]);
        assert!(series[0].label.contains(&summary_text(&r)));
    }

    // Spec: bins 2, 4, 8 Hz with 6 Hz omitted (bin spacing 2 Hz) give two curve runs, so no
    // line crosses the gap; only the first run has a legend label.
    #[test]
    fn omitted_bins_split_the_curve() {
        let r = result("Roll", vec![2.0, 4.0, 8.0], vec![0.9, 0.8, 0.7]);
        let (_, series) = axis_series(&r).expect("series");
        assert_eq!(series.len(), 4);
        assert_eq!(series[0].data, [(2.0, 0.9), (4.0, 0.8)]);
        assert_eq!(series[1].data, [(8.0, 0.7)]);
        assert!(!series[0].label.is_empty() && series[1].label.is_empty());
    }

    #[test]
    fn empty_result_has_no_series() {
        assert!(axis_series(&result("Roll", vec![], vec![])).is_none());
    }

    // Spec: when every bin lies above the plotted range there is no measurement, so no series
    // (and no reference-line-only PNG); a bin exactly at the limit is kept.
    #[test]
    fn bins_all_above_the_plot_range_have_no_series() {
        let above = COHERENCE_PLOT_MAX_HZ + 2.0;
        assert!(axis_series(&result("Roll", vec![above, above + 2.0], vec![0.5, 0.5])).is_none());
        assert!(axis_series(&result("Roll", vec![COHERENCE_PLOT_MAX_HZ], vec![0.5])).is_some());
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        let r = result("Roll", vec![above], vec![0.5]);
        plot_coherence(&[r], &root).expect("ok");
        assert!(!std::path::Path::new(&format!("{root}_Coherence_stacked.png")).exists());
    }

    // The PNG is written when an axis has data.
    #[test]
    fn png_is_written_for_one_axis() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        let r = result("Pitch", vec![2.0, 4.0, 6.0], vec![0.9, 0.5, 0.1]);
        plot_coherence(&[r], &root).expect("ok");
        assert!(std::path::Path::new(&format!("{root}_Coherence_stacked.png")).exists());
    }
}
