// tests/report_filter_config_test.rs
// Verifies that the "IMUF / Pseudo-Kalman" column is only rendered in the
// generated `*_report.md` Gyro Filter Configuration table when at least one
// axis actually has an IMUF filter configured (i.e. EmuFlight logs), and
// that the section is titled "Gyro Filter Configuration".

use std::fs;

use blackbox_analyzer::data_analysis::filter_response::{
    AllFilterConfigs, AxisFilterConfig, FilterConfig, FilterType, ImufFilterConfig,
    RpmFilterConfig, RpmFundamentalHz, RpmMotorFrequencies,
};
use blackbox_analyzer::report::{generate_markdown_report, FlightReport};

fn base_report(filter_config: Option<AllFilterConfigs>) -> FlightReport {
    FlightReport {
        root_name: "test_log".to_string(),
        sample_rate: Some(1000.0),
        trim_window: None,
        header_metadata: vec![],
        pd_ratios: [None, None, None],
        step_reports: vec![],
        optimal_p: [None, None, None],
        gyro_analysis: None,
        dterm_results: vec![],
        bode_results: vec![],
        motor_results: vec![],
        motor_desync_results: vec![],
        rc_command_steps: vec![],
        stick_distribution_results: vec![],
        png_links: vec![],
        skipped_plots: vec![],
        filter_config,
        dynamic_notch: None,
        rpm_filter: None,
        rpm_motor_frequencies: None,
        debug_fallback: false,
        debug_mode_name: None,
    }
}

fn lpf1_only_axis_config() -> AxisFilterConfig {
    AxisFilterConfig {
        lpf1: Some(FilterConfig {
            filter_type: FilterType::PT1,
            cutoff_hz: 250.0,
            q_factor: None,
            enabled: true,
        }),
        lpf2: None,
        dynamic_lpf1: None,
        imuf: None,
    }
}

#[test]
fn filter_config_table_omits_imuf_column_when_no_axis_has_imuf() {
    let mut fc = AllFilterConfigs::default();
    fc.gyro[0] = lpf1_only_axis_config();
    fc.gyro[1] = lpf1_only_axis_config();
    fc.gyro[2] = lpf1_only_axis_config();

    let report = base_report(Some(fc));
    let dir = tempfile::tempdir().expect("create temp dir");
    let out_path = dir.path().join("no_imuf_report.md");
    generate_markdown_report(&report, &out_path).expect("generate report");
    let contents = fs::read_to_string(&out_path).expect("read report");

    assert!(contents.contains("## Gyro Filter Configuration"));
    assert!(!contents.contains("IMUF / Pseudo-Kalman"));
    assert!(contents.contains("| Axis | LPF1 | LPF2 |"));
}

#[test]
fn filter_config_table_includes_imuf_column_when_an_axis_has_imuf() {
    let mut fc = AllFilterConfigs::default();
    fc.gyro[0] = AxisFilterConfig {
        lpf1: None,
        lpf2: None,
        dynamic_lpf1: None,
        imuf: Some(ImufFilterConfig {
            lowpass_cutoff_hz: 90.0,
            ptn_order: 1,
            q_factor: 200.0,
            revision: Some(256),
            pseudo_kalman_w: Some(4.0),
            effective_cutoff_hz: 90.0,
            enabled: true,
        }),
    };
    fc.gyro[1] = lpf1_only_axis_config();
    fc.gyro[2] = lpf1_only_axis_config();

    let report = base_report(Some(fc));
    let dir = tempfile::tempdir().expect("create temp dir");
    let out_path = dir.path().join("imuf_report.md");
    generate_markdown_report(&report, &out_path).expect("generate report");
    let contents = fs::read_to_string(&out_path).expect("read report");

    assert!(contents.contains("## Gyro Filter Configuration"));
    assert!(contents.contains("| Axis | LPF1 | LPF2 | IMUF / Pseudo-Kalman |"));
}

fn rpm_report(frequencies: Option<RpmMotorFrequencies>) -> String {
    // The section is only written when at least one axis has a filter configured
    let mut fc = AllFilterConfigs::default();
    fc.gyro[0] = lpf1_only_axis_config();
    let mut report = base_report(Some(fc));
    report.rpm_filter = Some(RpmFilterConfig {
        harmonics: 2,
        min_hz: 100.0,
        q_factor: 5.0,
        weights: vec![1.0, 1.0],
        motor_poles: Some(12),
        fade_range_hz: 50.0,
        lpf_hz: 150.0,
    });
    report.rpm_motor_frequencies = frequencies;
    let dir = tempfile::tempdir().expect("create temp dir");
    let out_path = dir.path().join("rpm_report.md");
    generate_markdown_report(&report, &out_path).expect("generate report");
    fs::read_to_string(&out_path).expect("read report")
}

#[test]
fn report_lists_rpm_filter_motor_frequency_ranges() {
    let contents = rpm_report(Some(RpmMotorFrequencies {
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
    }));
    assert!(contents.contains(
        "- **RPM Filter motor frequency:** 167–290 Hz (median 229) (all motors, 10th–90th percentile, from eRPM)"
    ));
    assert!(contents.contains(
        "- **RPM Filter harmonic ranges:** H1 167–290 Hz (median 229), H2 334–580 Hz (median 458)"
    ));
    assert!(contents
        .contains("- **RPM Filter per-motor frequency:** M0 160–280 Hz (median 220), M1 no eRPM"));
}

#[test]
fn report_says_rpm_motor_frequency_unavailable_without_erpm() {
    let contents = rpm_report(None);
    assert!(contents.contains(
        "- **RPM Filter motor frequency:** unavailable (log has no eRPM or motor_poles)"
    ));
    assert!(!contents.contains("harmonic ranges"));
}

#[test]
fn report_lists_rpm_filter_without_gyro_lowpass_fields() {
    // No axis has a gyro filter stage; the RPM filter must still be reported.
    let mut report = base_report(Some(AllFilterConfigs::default()));
    report.rpm_filter = Some(RpmFilterConfig {
        harmonics: 3,
        min_hz: 100.0,
        q_factor: 5.0,
        weights: vec![1.0, 1.0, 1.0],
        motor_poles: None,
        fade_range_hz: 50.0,
        lpf_hz: 150.0,
    });
    let dir = tempfile::tempdir().expect("create temp dir");
    let out_path = dir.path().join("rpm_only_report.md");
    generate_markdown_report(&report, &out_path).expect("generate report");
    let contents = fs::read_to_string(&out_path).expect("read report");
    assert!(contents.contains("## Gyro Filter Configuration"));
    assert!(!contents.contains("| Axis | LPF1 | LPF2 |"));
    assert!(contents.contains("- **RPM Filter:** 3 harmonics, Q=5, min 100 Hz"));
    assert!(contents.contains("- **RPM Filter motor frequency:** unavailable"));
}
