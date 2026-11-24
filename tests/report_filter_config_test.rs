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
use blackbox_analyzer::report::{generate_markdown_report, FlightReport, StepAxisReport};

fn base_report(filter_config: Option<AllFilterConfigs>) -> FlightReport {
    FlightReport {
        root_name: "test_log".to_string(),
        sample_rate: Some(1000.0),
        trim_window: None,
        header_metadata: vec![],
        pd_ratios: [None, None, None],
        pid_labels: Default::default(),
        d_ranges: Default::default(),
        step_reports: vec![],
        optimal_p: [None, None, None],
        optimal_p_skip: Default::default(),
        gyro_analysis: None,
        dterm_results: vec![],
        bode_results: vec![],
        motor_results: vec![],
        motor_desync_results: vec![],
        rc_command_steps: vec![],
        stick_distribution_results: vec![],
        motor_symmetry: Default::default(),
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
        "- **RPM Filter harmonic ranges:** Harmonic 1 167–290 Hz (median 229), Harmonic 2 334–580 Hz (median 458)"
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

#[test]
fn pid_tuning_table_shows_zero_ff_label() {
    let mut report = base_report(None);
    report.pid_labels = [
        "P:57 I:66 D:58 D-Min:58 D-Max:80 FF:0".to_string(),
        "P:59 I:69 D:72 D-Min:44 D-Max:90 FF:0".to_string(),
        String::new(),
    ];
    let dir = tempfile::tempdir().expect("create temp dir");
    let out_path = dir.path().join("pid_report.md");
    generate_markdown_report(&report, &out_path).expect("generate report");
    let contents = fs::read_to_string(&out_path).expect("read report");

    assert!(contents.contains("| Axis | PID | P:D Ratio |"));
    assert!(contents.contains("| Roll | P:57 I:66 D:58 D-Min:58 D-Max:80 FF:0 | N/A |"));
    assert!(contents.contains("| Yaw (informational) | N/A | N/A |"));
}

fn step_axis(hint: Option<String>) -> StepAxisReport {
    StepAxisReport {
        axis_name: "Roll",
        peak_value: 1.01,
        assessment: "Near optimal",
        current_pd_ratio: 1.2,
        conservative: None,
        moderate: None,
        aggressive: None,
        d_note: None,
        near_optimal_hint: hint,
        setpoint_authority_name: None,
        setpoint_authority_mean: None,
        warnings: Vec::new(),
    }
}

#[test]
fn near_optimal_hint_replaces_the_none_line_in_the_report() {
    let dir = tempfile::tempdir().expect("create temp dir");

    let mut report = base_report(None);
    report.step_reports = vec![step_axis(Some(
        "D-Min≈19, D-Max≈29 [optional D−1]".to_string(),
    ))];
    let with_hint = dir.path().join("hint_report.md");
    generate_markdown_report(&report, &with_hint).expect("generate report");
    let contents = fs::read_to_string(&with_hint).expect("read report");
    assert!(
        contents.contains("- **Recommendation (conservative):** D-Min≈19, D-Max≈29 [optional D−1]")
    );
    assert!(!contents.contains("No obvious tuning adjustments needed"));

    let mut report = base_report(None);
    report.step_reports = vec![step_axis(None)];
    let without = dir.path().join("none_report.md");
    generate_markdown_report(&report, &without).expect("generate report");
    let contents = fs::read_to_string(&without).expect("read report");
    assert!(contents.contains("- **Recommendation:** No obvious tuning adjustments needed"));
}

#[test]
fn optimal_p_section_shows_the_d_range_like_the_plot() {
    use blackbox_analyzer::data_analysis::optimal_p_estimation::OptimalPAnalysis;

    // P 40, D 20, P:D target 2.0; a Td far above the target gives an Increase P recommendation.
    let samples: Vec<f64> = (0..40).map(|i| 60.0 + (i % 5) as f64).collect();
    let analysis =
        OptimalPAnalysis::analyze(&samples, 40, Some(20), None, Some(2.0), Some((25.0, 3.0)))
            .expect("analysis from synthetic Td samples");
    let dir = tempfile::tempdir().expect("create temp dir");

    let mut report = base_report(None);
    report.optimal_p[0] = Some(analysis.clone());
    report.d_ranges[0] = Some((20, 30));
    let ranged = dir.path().join("ranged_report.md");
    generate_markdown_report(&report, &ranged).expect("generate report");
    let contents = fs::read_to_string(&ranged).expect("read report");
    assert!(
        contents.contains("- **Current D:** D-Min 20, D-Max 30"),
        "{contents}"
    );
    let rec = contents
        .lines()
        .find(|l| l.starts_with("- **Recommendation:** Increase P to"))
        .expect("an Increase P recommendation line");
    // The report carries the shared helpers' exact text for the recommended P.
    let recommended_p = match &analysis.recommendation {
        blackbox_analyzer::data_analysis::optimal_p_estimation::PRecommendation::Increase {
            conservative_p,
            ..
        } => *conservative_p,
        other => panic!("expected an Increase P recommendation, got {other:?}"),
    };
    let expected_suffix = analysis.d_target_suffix(recommended_p, Some((20, 30)));
    assert!(expected_suffix.starts_with(", D-Min≈"), "{expected_suffix}");
    assert!(
        rec.starts_with(&format!(
            "- **Recommendation:** Increase P to {recommended_p} ({:+}){expected_suffix} — ",
            recommended_p as i64 - 40
        )),
        "{rec}"
    );
    let expected_reliability = format!(
        "- **{}:** {}",
        analysis.td_stats.reliability_label(),
        analysis.td_stats.reliability_details()
    );
    assert!(
        contents.contains(&expected_reliability),
        "{expected_reliability}"
    );

    report.d_ranges[0] = None;
    let single = dir.path().join("single_report.md");
    generate_markdown_report(&report, &single).expect("generate report");
    let contents = fs::read_to_string(&single).expect("read report");
    assert!(contents.contains("- **Current D:** 20"));
    let rec = contents
        .lines()
        .find(|l| l.starts_with("- **Recommendation:** Increase P to"))
        .expect("an Increase P recommendation line");
    assert!(rec.contains(", D≈") && !rec.contains("D-Min"), "{rec}");
}

#[test]
fn near_optimal_hint_report_text_matches_the_shared_helper_for_roll_and_pitch() {
    use blackbox_analyzer::data_input::pid_metadata::parse_pid_metadata;

    // BF 4.5: roll has a range (20 to 30); pitch has d_min 0, so it shows a single D.
    let headers: Vec<(String, String)> = [
        ("Firmware revision", "Betaflight 4.5.0"),
        ("rollPID", "40,50,30"),
        ("pitchPID", "42,52,40"),
        ("d_min", "20,0,0"),
    ]
    .iter()
    .map(|(k, v)| (k.to_string(), v.to_string()))
    .collect();
    let pid = parse_pid_metadata(&headers);
    let on = pid.is_dmax_enabled();
    let roll_hint = pid.roll.near_optimal_d_hint(on).expect("roll hint");
    let pitch_hint = pid.pitch.near_optimal_d_hint(on).expect("pitch hint");
    assert!(roll_hint.contains("D-Min≈"), "{roll_hint}");
    assert!(
        pitch_hint.starts_with("D≈") && !pitch_hint.contains("D-Min"),
        "{pitch_hint}"
    );

    let mut report = base_report(None);
    let mut roll = step_axis(Some(roll_hint.clone()));
    roll.axis_name = "Roll";
    let mut pitch = step_axis(Some(pitch_hint.clone()));
    pitch.axis_name = "Pitch";
    report.step_reports = vec![roll, pitch];
    let dir = tempfile::tempdir().expect("create temp dir");
    let path = dir.path().join("two_axis_report.md");
    generate_markdown_report(&report, &path).expect("generate report");
    let contents = fs::read_to_string(&path).expect("read report");
    // Each hint sits in its own axis section, not just anywhere in the report.
    let section = |axis: &str| -> String {
        let head = format!("### {axis}\n");
        let start = contents.find(&head).expect("axis section");
        let rest = &contents[start + head.len()..];
        rest.split("\n### ").next().unwrap_or(rest).to_string()
    };
    let roll_section = section("Roll");
    let pitch_section = section("Pitch");
    assert!(roll_section.contains(&format!("- **Recommendation (conservative):** {roll_hint}")));
    assert!(!roll_section.contains(&pitch_hint));
    assert!(pitch_section.contains(&format!(
        "- **Recommendation (conservative):** {pitch_hint}"
    )));
    assert!(!pitch_section.contains(&roll_hint));
}

#[test]
fn optimal_p_source_line_uses_the_shared_summary() {
    use blackbox_analyzer::data_analysis::optimal_p_estimation::OptimalPAnalysis;

    let samples: Vec<f64> = (0..40).map(|i| 60.0 + (i % 5) as f64).collect();
    let mut analysis =
        OptimalPAnalysis::analyze(&samples, 40, Some(20), None, Some(2.0), Some((25.0, 3.0)))
            .expect("analysis from synthetic Td samples");
    analysis.source_files = 11;
    analysis.source_events = 94;
    let expected = analysis.source_summary();
    assert_eq!(expected, "11 flights, 94 punches");

    let mut report = base_report(None);
    report.optimal_p[1] = Some(analysis);
    let dir = tempfile::tempdir().expect("create temp dir");
    let path = dir.path().join("source_report.md");
    generate_markdown_report(&report, &path).expect("generate report");
    let contents = fs::read_to_string(&path).expect("read report");
    assert!(
        contents.contains(&format!("- **Source:** {expected}")),
        "{contents}"
    );
}

// The report carries the shared decision's text: when every tier names the current D, the
// report prints the shared note, and no tier line.
#[test]
fn same_d_report_carries_the_shared_note_and_drops_the_tier() {
    use blackbox_analyzer::constants::{D_TARGET_WITHIN_ROUNDING_NOTE, PEAK_ACCEPTABLE_MULTIPLIER};
    use blackbox_analyzer::data_input::pid_metadata::parse_pid_metadata;

    let headers: Vec<(String, String)> = [
        ("Firmware revision", "Betaflight 4.4.0"),
        ("rollPID", "40,50,12"),
    ]
    .iter()
    .map(|(k, v)| (k.to_string(), v.to_string()))
    .collect();
    let pid = parse_pid_metadata(&headers);
    let on = pid.is_dmax_enabled();
    let ratio = pid.roll.calculate_pd_ratio().unwrap() * PEAK_ACCEPTABLE_MULTIPLIER;
    let target = pid.roll.calculate_goal_d_with_range(ratio, on);
    let tiers = pid.roll.d_tier_visibility(Some(target), None, None, on);
    assert!(tiers.within_rounding_note && !tiers.conservative);

    let mut axis = step_axis(None);
    axis.assessment = "Acceptable";
    axis.d_note = tiers
        .within_rounding_note
        .then_some(D_TARGET_WITHIN_ROUNDING_NOTE);
    let mut report = base_report(None);
    report.step_reports = vec![axis];
    let dir = tempfile::tempdir().expect("create temp dir");
    let path = dir.path().join("same_d_report.md");
    generate_markdown_report(&report, &path).expect("generate report");
    let contents = fs::read_to_string(&path).expect("read report");
    assert!(
        contents.contains(&format!(
            "- **Recommendation:** {D_TARGET_WITHIN_ROUNDING_NOTE}"
        )),
        "{contents}"
    );
    assert!(!contents.contains("Recommendation (conservative)"));
    assert!(!contents.contains("No obvious tuning adjustments needed"));
}
