// src/plot_functions/plot_motor_symmetry.rs

use plotters::prelude::*;
use std::error::Error;

use crate::constants::{
    LEGEND_BACKGROUND_OPACITY, LINE_WIDTH_PLOT, MOTOR_SYMMETRY_BOX_COLOR,
    MOTOR_SYMMETRY_BOX_WIDTH_FRACTION, MOTOR_SYMMETRY_MEAN_COLOR, MOTOR_SYMMETRY_MEAN_MARKER_SIZE,
    MOTOR_SYMMETRY_MEDIAN_COLOR, MOTOR_SYMMETRY_MEDIAN_LINE_WIDTH, MOTOR_SYMMETRY_SLOT_HALF_WIDTH,
    MOTOR_SYMMETRY_Y_MIN_SPAN, MOTOR_SYMMETRY_Y_PADDING_FRACTION, PLOT_HEIGHT, PLOT_WIDTH,
};
use crate::data_analysis::motor_symmetry::{
    imbalance_summary_text, MotorCommandStats, MotorSymmetryResult,
};
use crate::plot_framework::record_written_this_run;

/// Y-axis range covering every whisker and mean marker, padded by `MOTOR_SYMMETRY_Y_PADDING_FRACTION` of the
/// span and at least `MOTOR_SYMMETRY_Y_MIN_SPAN` wide. `None` when `motors` is empty.
pub fn y_axis_range(motors: &[MotorCommandStats]) -> Option<(f64, f64)> {
    if motors.is_empty() {
        return None;
    }
    let lo = motors
        .iter()
        .map(|m| m.whisker_low().min(m.mean))
        .fold(f64::INFINITY, f64::min);
    let hi = motors
        .iter()
        .map(|m| m.whisker_high().max(m.mean))
        .fold(f64::NEG_INFINITY, f64::max);
    let span = (hi - lo).max(MOTOR_SYMMETRY_Y_MIN_SPAN);
    let center = (hi + lo) / 2.0;
    let half = span / 2.0 + span * MOTOR_SYMMETRY_Y_PADDING_FRACTION;
    Some((center - half, center + half))
}

/// Plot title: fixed name plus the shared imbalance summary (same text as the console/report).
pub fn plot_title(result: &MotorSymmetryResult) -> String {
    format!(
        "Motor Command Distribution - {}",
        imbalance_summary_text(result)
    )
}

/// Generates one box plot of the per-motor command distribution (box P25-P75, line at the
/// median, whiskers P5-P95, red marker at the mean). Writes no file when no motor has samples.
pub fn plot_motor_symmetry(
    result: &MotorSymmetryResult,
    root_name: &str,
) -> Result<(), Box<dyn Error>> {
    let output_file = format!("{root_name}_Motor_Symmetry.png");

    let Some((y_min, y_max)) = y_axis_range(&result.motors) else {
        println!("\nINFO: Skipping Motor Symmetry Plot: No motor data available.");
        return Ok(());
    };
    let slot_count = result.motors.iter().map(|m| m.motor_idx).max().unwrap_or(0) + 1;

    println!("\n--- Generating Motor Symmetry Plot ---");

    let root_area = BitMapBackend::new(&output_file, (PLOT_WIDTH, PLOT_HEIGHT)).into_drawing_area();
    root_area.fill(&WHITE)?;
    root_area.draw(&Text::new(
        root_name,
        (10, 10),
        crate::font_config::FONT_TUPLE_MAIN_TITLE
            .into_font()
            .color(&BLACK),
    ))?;
    let margined_root_area = root_area.margin(50, 5, 5, 5);

    let mut chart = ChartBuilder::on(&margined_root_area)
        .caption(
            plot_title(result),
            crate::font_config::FONT_TUPLE_CHART_TITLE,
        )
        .margin(5)
        .x_label_area_size(50)
        .y_label_area_size(70)
        .build_cartesian_2d(
            -MOTOR_SYMMETRY_SLOT_HALF_WIDTH..slot_count as f64 - MOTOR_SYMMETRY_SLOT_HALF_WIDTH,
            y_min..y_max,
        )?;

    chart
        .configure_mesh()
        .disable_x_mesh()
        .x_desc("Motor")
        .y_desc("Motor command (log units)")
        .x_labels(slot_count + 1)
        .x_label_formatter(&|x| {
            if *x >= 0.0 && (x - x.round()).abs() < f64::EPSILON * 8.0 {
                format!("{}", x.round() as usize)
            } else {
                String::new()
            }
        })
        .y_labels(10)
        .light_line_style(WHITE.mix(0.7))
        .label_style(crate::font_config::FONT_TUPLE_AXIS_LABEL)
        .draw()?;

    let half_width = MOTOR_SYMMETRY_BOX_WIDTH_FRACTION / 2.0;
    let box_style = ShapeStyle::from(MOTOR_SYMMETRY_BOX_COLOR).stroke_width(LINE_WIDTH_PLOT);
    let median_style = ShapeStyle::from(MOTOR_SYMMETRY_MEDIAN_COLOR)
        .stroke_width(MOTOR_SYMMETRY_MEDIAN_LINE_WIDTH);

    for m in &result.motors {
        let x = m.motor_idx as f64;
        // Whisker (P5-P95), box (P25-P75), median line, mean marker.
        chart.draw_series(std::iter::once(PathElement::new(
            vec![(x, m.whisker_low()), (x, m.q1())],
            box_style,
        )))?;
        chart.draw_series(std::iter::once(PathElement::new(
            vec![(x, m.q3()), (x, m.whisker_high())],
            box_style,
        )))?;
        chart.draw_series(std::iter::once(Rectangle::new(
            [(x - half_width, m.q1()), (x + half_width, m.q3())],
            MOTOR_SYMMETRY_BOX_COLOR
                .mix(LEGEND_BACKGROUND_OPACITY)
                .filled(),
        )))?;
        chart.draw_series(std::iter::once(Rectangle::new(
            [(x - half_width, m.q1()), (x + half_width, m.q3())],
            box_style,
        )))?;
        chart.draw_series(std::iter::once(PathElement::new(
            vec![(x - half_width, m.median()), (x + half_width, m.median())],
            median_style,
        )))?;
        chart.draw_series(std::iter::once(Circle::new(
            (x, m.mean),
            MOTOR_SYMMETRY_MEAN_MARKER_SIZE,
            MOTOR_SYMMETRY_MEAN_COLOR.filled(),
        )))?;
    }

    // Legend entries (drawn off-data so they carry no extra marks).
    chart
        .draw_series(std::iter::empty::<PathElement<(f64, f64)>>())?
        .label("Box P25-P75, whiskers P5-P95, black line median")
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], median_style));
    chart
        .draw_series(std::iter::empty::<PathElement<(f64, f64)>>())?
        .label("Mean")
        .legend(move |(x, y)| {
            Circle::new(
                (x + 10, y),
                MOTOR_SYMMETRY_MEAN_MARKER_SIZE,
                MOTOR_SYMMETRY_MEAN_COLOR.filled(),
            )
        });
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(LEGEND_BACKGROUND_OPACITY))
        .border_style(BLACK)
        .label_font(crate::font_config::FONT_TUPLE_LEGEND)
        .draw()?;

    root_area.present()?;
    record_written_this_run(&output_file);
    println!("  Plot saved as '{output_file}'.");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_analysis::motor_symmetry::analyze_motor_symmetry;
    use crate::data_input::log_data::LogRowData;

    fn rows(values: &[f64], n: usize) -> Vec<LogRowData> {
        (0..n)
            .map(|_| LogRowData {
                motors: values.iter().map(|v| Some(*v)).collect(),
                ..Default::default()
            })
            .collect()
    }

    // Spec: no motors, no PNG, success.
    #[test]
    fn no_motor_data_creates_no_png() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        plot_motor_symmetry(&MotorSymmetryResult::default(), &root).expect("ok");
        assert!(!std::path::Path::new(&format!("{root}_Motor_Symmetry.png")).exists());
    }

    // Spec: equal motors (zero-height boxes) still render a file.
    #[test]
    fn equal_motors_create_a_png() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path().join("log").to_string_lossy().into_owned();
        let result = analyze_motor_symmetry(&rows(&[500.0; 6], 3));
        plot_motor_symmetry(&result, &root).expect("ok");
        assert!(std::path::Path::new(&format!("{root}_Motor_Symmetry.png")).exists());
    }

    // Spec: the Y range covers the whiskers with padding and never collapses.
    #[test]
    fn y_range_covers_whiskers_and_has_minimum_span() {
        let result = analyze_motor_symmetry(&rows(&[100.0, 200.0], 3));
        let (lo, hi) = y_axis_range(&result.motors).expect("range");
        assert!(lo < 100.0 && hi > 200.0);
        let flat = analyze_motor_symmetry(&rows(&[500.0, 500.0], 3));
        let (lo, hi) = y_axis_range(&flat.motors).expect("range");
        assert!(hi - lo >= MOTOR_SYMMETRY_Y_MIN_SPAN);
        assert!(lo < 500.0 && hi > 500.0);
        assert!(y_axis_range(&[]).is_none());

        // Skewed motor: 96 samples at 100 and 4 at 1000 give both whiskers 100 but mean 136.
        let mut skewed = rows(&[100.0], 96);
        skewed.extend(rows(&[1000.0], 4));
        let result = analyze_motor_symmetry(&skewed);
        let m = &result.motors[0];
        assert_eq!((m.whisker_low(), m.whisker_high()), (100.0, 100.0));
        assert!((m.mean - 136.0).abs() < 1e-9);
        let (lo, hi) = y_axis_range(&result.motors).expect("range");
        assert!(lo < 100.0 && hi > m.mean);
    }

    // Spec: the title carries the shared imbalance text.
    #[test]
    fn title_carries_the_shared_summary() {
        let result = analyze_motor_symmetry(&rows(&[100.0, 120.0], 2));
        assert!(plot_title(&result).ends_with(&imbalance_summary_text(&result)));
    }
}
