// src/plot_functions/plot_dynamic_notch_activity.rs

use std::error::Error;

use crate::constants::{
    COLOR_DYN_NOTCH_ACTIVITY, DYN_NOTCH_ACTIVITY_MAX_NOTCHES, DYN_NOTCH_ACTIVITY_MIN_X_SPAN_S,
    DYN_NOTCH_ACTIVITY_MIN_Y_SPAN_HZ, DYN_NOTCH_ACTIVITY_Y_PADDING_FRACTION, LINE_WIDTH_PLOT,
};
use crate::data_analysis::dynamic_notch_activity::{
    debug_axis_text, legend_text, DynamicNotchActivity,
};
use crate::plot_framework::{draw_single_chart_plot, PlotConfig, PlotSeries};

/// Debug mode name shown in the title: the data comes from debug channels.
const DEBUG_MODE_NAME: &str = "FFT_FREQ";

/// Y-axis range in Hz: the header's dynamic notch range widened to include every plotted centre
/// (a centre can sit outside the header range only if the header and log disagree), or the data
/// range alone without a header range. Padded, and never narrower than the minimum span.
pub fn y_range_hz(activity: &DynamicNotchActivity) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for n in &activity.notches {
        lo = lo.min(n.min_hz);
        hi = hi.max(n.max_hz);
    }
    if let Some((min_hz, max_hz)) = activity.range_hz {
        lo = lo.min(min_hz);
        hi = hi.max(max_hz);
    }
    let pad = (hi - lo) * DYN_NOTCH_ACTIVITY_Y_PADDING_FRACTION;
    let (lo, hi) = (lo - pad, hi + pad);
    let missing = DYN_NOTCH_ACTIVITY_MIN_Y_SPAN_HZ - (hi - lo);
    if missing > 0.0 {
        (lo - missing / 2.0, hi + missing / 2.0)
    } else {
        (lo, hi)
    }
}

/// Plot series of the activity: one coloured line per notch, one `PlotSeries` per run of
/// consecutive centres. Only the first run of a notch carries the legend label.
fn build_series(activity: &DynamicNotchActivity) -> Vec<PlotSeries> {
    let mut series = Vec::new();
    for n in &activity.notches {
        let color =
            *COLOR_DYN_NOTCH_ACTIVITY[(n.notch - 1).min(DYN_NOTCH_ACTIVITY_MAX_NOTCHES - 1)];
        for (i, seg) in n.segments.iter().enumerate() {
            series.push(PlotSeries {
                data: seg.clone(),
                label: if i == 0 {
                    legend_text(n)
                } else {
                    String::new()
                },
                color,
                stroke_width: LINE_WIDTH_PLOT,
            });
        }
    }
    series
}

/// Plots the notch centre frequencies against time. Writes nothing when no notch has a segment.
pub fn plot_dynamic_notch_activity(
    activity: &DynamicNotchActivity,
    root_name: &str,
) -> Result<(), Box<dyn Error>> {
    let output_file = format!("{root_name}_Dynamic_Notch_Activity.png");
    let plot_type_name = "Dynamic Notch Activity";
    let series = build_series(activity);

    let mut t_min = f64::INFINITY;
    let mut t_max = f64::NEG_INFINITY;
    for s in &series {
        for (t, _) in &s.data {
            t_min = t_min.min(*t);
            t_max = t_max.max(*t);
        }
    }
    if series.is_empty() {
        return draw_single_chart_plot(
            &output_file,
            root_name,
            plot_type_name,
            &PlotConfig {
                title: String::new(),
                x_range: 0.0..0.0,
                y_range: 0.0..0.0,
                series,
                x_label: String::new(),
                y_label: String::new(),
                peaks: vec![],
                peak_label_threshold: None,
                peak_label_format_string: None,
                frequency_ranges: None,
            },
        );
    }
    // One sample (or one time stamp) has no span: give the axis the minimum width.
    let t_max = t_max.max(t_min + DYN_NOTCH_ACTIVITY_MIN_X_SPAN_S);
    let (y_lo, y_hi) = y_range_hz(activity);
    let title = super::format_debug_suffix(
        &format!(
            "Dynamic Notch Centre Frequency ({})",
            debug_axis_text(activity)
        ),
        true,
        Some(DEBUG_MODE_NAME),
    );
    let config = PlotConfig {
        title,
        x_range: t_min..t_max,
        y_range: y_lo..y_hi,
        series,
        x_label: "Time (s)".to_string(),
        y_label: "Notch Centre Frequency (Hz)".to_string(),
        peaks: vec![],
        peak_label_threshold: None,
        peak_label_format_string: None,
        frequency_ranges: None,
    };
    draw_single_chart_plot(&output_file, root_name, plot_type_name, &config)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_analysis::dynamic_notch_activity::{
        analyze_dynamic_notch_activity, FftFreqLayout,
    };
    use crate::data_input::log_data::LogRowData;

    fn activity(centres: &[f64], range: Option<(f64, f64)>) -> DynamicNotchActivity {
        let rows: Vec<LogRowData> = centres
            .iter()
            .enumerate()
            .map(|(i, c)| LogRowData {
                time_sec: Some(i as f64),
                debug: [Some(-3.0), Some(*c), Some(0.0), Some(0.0)],
                ..Default::default()
            })
            .collect();
        analyze_dynamic_notch_activity(&rows, &[], FftFreqLayout::GyroFirst, range)
            .expect("activity")
    }

    // Spec: header range 100..600 (span 500), padding fraction p on each side, centres inside.
    #[test]
    fn y_range_covers_header_range_with_padding() {
        let a = activity(&[200.0, 300.0], Some((100.0, 600.0)));
        let (lo, hi) = y_range_hz(&a);
        let pad = 500.0 * DYN_NOTCH_ACTIVITY_Y_PADDING_FRACTION;
        assert_eq!((lo, hi), (100.0 - pad, 600.0 + pad));
    }

    #[test]
    fn y_range_widens_to_a_centre_outside_the_header_range() {
        let a = activity(&[50.0, 300.0], Some((100.0, 600.0)));
        let (lo, hi) = y_range_hz(&a);
        assert!(lo < 50.0 && hi > 600.0, "{lo} {hi}");
    }

    // A constant centre has span 0: the range still has the minimum span and holds the value.
    #[test]
    fn y_range_of_a_constant_centre_has_minimum_span() {
        let a = activity(&[250.0, 250.0], None);
        let (lo, hi) = y_range_hz(&a);
        assert!(
            (hi - lo - DYN_NOTCH_ACTIVITY_MIN_Y_SPAN_HZ).abs() < 1e-9,
            "{lo} {hi}"
        );
        assert!(lo < 250.0 && 250.0 < hi);
    }

    // Only the first run of a notch carries the legend label.
    #[test]
    fn only_the_first_segment_has_a_legend_label() {
        let a = activity(&[100.0, 0.0, 200.0, 0.0, 300.0], None);
        let s = build_series(&a);
        assert_eq!(s.len(), 3);
        assert_eq!(s[0].label, "Notch 1 (median 200 Hz)");
        assert!(s[1].label.is_empty() && s[2].label.is_empty());
    }

    #[test]
    fn plot_is_written_for_one_sample_and_not_for_no_segments() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        let png = format!("{root}_Dynamic_Notch_Activity.png");

        let one = activity(&[321.0], Some((100.0, 600.0)));
        plot_dynamic_notch_activity(&one, &root).expect("plot");
        assert!(std::path::Path::new(&png).exists());

        std::fs::remove_file(&png).expect("remove");
        let mut empty = one.clone();
        empty.notches.clear();
        plot_dynamic_notch_activity(&empty, &root).expect("plot");
        assert!(!std::path::Path::new(&png).exists());
    }
}
