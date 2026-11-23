// src/data_input/pid_metadata.rs

// Import AXIS_COUNT for non-test builds
#[cfg(not(test))]
use crate::axis_names::AXIS_COUNT;

// For test builds (when included via include!()), define locally to match axis_names
#[cfg(test)]
const AXIS_COUNT: usize = 3; // Keep in sync with crate::axis_names::AXIS_COUNT

// Axis indices for array access consistency
const ROLL_AXIS: usize = 0;
const PITCH_AXIS: usize = 1;
const YAW_AXIS: usize = 2;

/// Firmware type detection for appropriate terminology
#[derive(Debug, Clone, PartialEq, Default)]
pub enum FirmwareType {
    Betaflight,
    EmuFlight,
    Inav,
    #[default]
    Unknown,
}

/// Aggressive P:D target with its D values, from `AxisPid::aggressive_recommendation`
#[derive(Debug, Clone, Copy)]
pub struct AggressiveRecommendation {
    pub pd_ratio: f64,
    pub d: Option<u32>,
    pub d_min: Option<u32>,
    pub d_max: Option<u32>,
}

/// Recommended D target of one tier: base D, D-Min, D-Max (as `calculate_goal_d_with_range` returns them).
pub type DTarget = (Option<u32>, Option<u32>, Option<u32>);

/// Which P:D recommendation tiers an axis shows, and whether the "no D change" note replaces them.
/// From `AxisPid::d_tier_visibility`; console, report and plot legend all read it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DTierVisibility {
    pub conservative: bool,
    pub moderate: bool,
    pub aggressive: bool,
    /// True when a conservative tier exists but every existing tier names the current D.
    pub within_rounding_note: bool,
}

/// True when a recommended D target names the values the axis already has, so it is not a
/// recommendation. Mirrors `format_d_target`: with `range_shown` the D-Min and D-Max ends
/// that are known are compared (a changed end makes the target a change); otherwise base D.
pub fn d_target_is_current(current: DTarget, target: DTarget, range_shown: bool) -> bool {
    let (cur_d, cur_min, cur_max) = current;
    let (d, d_min, d_max) = target;
    if range_shown {
        (d_min.is_none() || d_min == cur_min) && (d_max.is_none() || d_max == cur_max)
    } else {
        d.is_some() && d == cur_d
    }
}

/// True when step-response P:D advice is shown. Console, plot legend and report all use this
/// one decision. "Acceptable" and "Undershoot" always pass (their delta is small by design);
/// other zones need a P:D change above `PD_RATIO_MIN_CHANGE_THRESHOLD`.
pub fn pd_advice_passes_change_gate(
    assessment: &str,
    current_pd_ratio: f64,
    recommended_ratio: f64,
) -> bool {
    assessment == "Acceptable"
        || assessment == "Undershoot"
        || (recommended_ratio - current_pd_ratio).abs()
            > crate::constants::PD_RATIO_MIN_CHANGE_THRESHOLD
}

/// PID values for a single axis
#[derive(Debug, Clone, Default)]
pub struct AxisPid {
    pub p: Option<u32>,
    pub i: Option<u32>,
    pub d: Option<u32>,
    pub d_min: Option<u32>, // D-Min for Betaflight
    pub d_max: Option<u32>, // D-Max for newer Betaflight
    pub ff: Option<u32>,
}

impl AxisPid {
    /// Calculate P:D ratio (P divided by D)
    /// Returns None if P or D is missing, or if D is 0
    /// Uses base D value (not D-Min) when D-Min/D-Max is configured
    /// This represents the "setpoint" D value that D-Min/D-Max modulates around
    pub fn calculate_pd_ratio(&self) -> Option<f64> {
        let p_val = self.p?;

        // Use base D value - this is what the user sets and what D-Min/D-Max modulate around
        // When D-Min/D-Max are disabled (both 0 or one is 0), base D is the actual D
        // When D-Min/D-Max are enabled, base D is still the reference value for tuning
        let d_val = self.d?;

        if d_val == 0 {
            return None; // Avoid division by zero
        }

        Some((p_val as f64) / (d_val as f64))
    }

    /// Calculate the recommended D value to achieve a target P:D ratio
    /// Returns None if P is missing
    /// Default target ratio is 1.4 (recommended for good damping)
    pub fn calculate_goal_d_for_ratio(&self, target_ratio: f64) -> Option<u32> {
        let p_val = self.p?;

        if target_ratio <= 0.0 {
            return None; // Invalid target ratio
        }

        // Calculate D = P / target_ratio
        let recommended_d = (p_val as f64) / target_ratio;
        Some(recommended_d.round() as u32)
    }

    /// Calculate recommended D, D-Min, and D-Max values to achieve a target P:D ratio
    /// Maintains the same proportional relationship between base D, D-Min, and D-Max
    /// Returns (base_d, d_min, d_max) tuple - any value can be None if not calculable
    ///
    /// Note: For Betaflight < 4.6, base D = D-Max (maximum). For BF >= 4.6, base D = D-Min (minimum).
    /// This function calculates proportionally regardless of firmware version.
    ///
    /// When this axis has no valid D-Min/D-Max pair (see `d_range_for_display`), returns
    /// (base_d, None, None) regardless of d_min/d_max field presence.
    pub fn calculate_goal_d_with_range(
        &self,
        target_ratio: f64,
        dmax_enabled: bool,
    ) -> (Option<u32>, Option<u32>, Option<u32>) {
        let recommended_base_d = self.calculate_goal_d_for_ratio(target_ratio);

        // If we can't calculate base D or don't have current D, return early
        let Some(rec_base) = recommended_base_d else {
            return (None, None, None);
        };
        let Some(current_base) = self.d else {
            return (recommended_base_d, None, None);
        };

        if current_base == 0 {
            return (recommended_base_d, None, None);
        }

        // Without a valid D-Min/D-Max pair on this axis, only return base D
        if self.d_range_for_display(dmax_enabled).is_none() {
            return (recommended_base_d, None, None);
        }

        // A recommended base D that rounds to 0 has no representable range: base D only.
        if rec_base == 0 {
            return (recommended_base_d, None, None);
        }

        // Calculate the scaling factor
        let scale_factor = (rec_base as f64) / (current_base as f64);

        // Calculate recommended D-Min if present
        let rec_d_min = self.d_min.map(|current_min| {
            if current_min > 0 {
                // A recommended D-Min of 0 would switch dynamic D off.
                (((current_min as f64) * scale_factor).round() as u32).max(1)
            } else {
                0
            }
        });

        // Calculate recommended D-Max
        let rec_d_max = if let Some(current_max) = self.d_max {
            // D-Max field exists (BF 4.6+)
            Some(if current_max > 0 {
                ((current_max as f64) * scale_factor).round() as u32
            } else {
                0
            })
        } else if self.d_min.is_some() {
            // D-Max field missing but D-Min exists (older firmware < 4.6)
            // In old firmware, base D IS the D-Max, so use recommended base D as D-Max
            recommended_base_d // Base D = D-Max in old firmware
        } else {
            // No D-Min/D-Max system at all
            None
        };

        (recommended_base_d, rec_d_min, rec_d_max)
    }

    /// True when this axis has a dynamic D range: 0 < D-Min < D-Max.
    ///
    /// Matches the firmware activation rule on every Betaflight branch:
    /// - 4.0 to 4.5 (`pid_init.c` / `pid.c`): `d_min > 0 && d_min < D`, D is the maximum.
    /// - 2025.12 and master (`pid_init.c`): `D > 0 && d_max > D`, D is the minimum.
    ///
    /// `parse_pid_metadata` fills the missing end of the range from base D, so this one
    /// check covers both layouts.
    pub fn has_dynamic_d_range(&self) -> bool {
        matches!((self.d_min, self.d_max), (Some(min), Some(max)) if min > 0 && min < max)
    }

    /// Aggressive P:D recommendation for the significant-overshoot zone: the ratio is the
    /// current P:D times `PD_RATIO_AGGRESSIVE_MULTIPLIER`, with the matching D (and D-Min/D-Max
    /// on an axis with a valid range). Single source for the plot legend, console and report.
    pub fn aggressive_recommendation(
        &self,
        current_pd_ratio: f64,
        dmax_enabled: bool,
    ) -> AggressiveRecommendation {
        let pd_ratio = current_pd_ratio * crate::constants::PD_RATIO_AGGRESSIVE_MULTIPLIER;
        let (d, d_min, d_max) = self.calculate_goal_d_with_range(pd_ratio, dmax_enabled);
        AggressiveRecommendation {
            pd_ratio,
            d,
            d_min,
            d_max,
        }
    }

    /// True when `target` equals this axis' current D (or D-Min and D-Max when the target is
    /// shown as a range). `d_target_is_current` holds the rule.
    pub fn target_is_current_d(&self, target: DTarget, dmax_enabled: bool) -> bool {
        let range_shown = dmax_enabled && (target.1.is_some() || target.2.is_some());
        d_target_is_current((self.d, self.d_min, self.d_max), target, range_shown)
    }

    /// Single decision for the P:D tiers of one axis, used by console, report and plot legend.
    /// A tier whose D target equals the current D is not shown. When a conservative tier
    /// exists and every existing tier is dropped, `within_rounding_note` is set instead.
    pub fn d_tier_visibility(
        &self,
        conservative: Option<DTarget>,
        moderate: Option<DTarget>,
        aggressive: Option<DTarget>,
        dmax_enabled: bool,
    ) -> DTierVisibility {
        let shown =
            |t: Option<DTarget>| t.is_some_and(|t| !self.target_is_current_d(t, dmax_enabled));
        let (conservative_shown, moderate_shown, aggressive_shown) =
            (shown(conservative), shown(moderate), shown(aggressive));
        DTierVisibility {
            conservative: conservative_shown,
            moderate: moderate_shown,
            aggressive: aggressive_shown,
            within_rounding_note: conservative.is_some()
                && !(conservative_shown || moderate_shown || aggressive_shown),
        }
    }

    /// Optional D-1 hint for an axis in the near-optimal peak zone. Single source for the
    /// plot legend, console and report: base D, or D-Min and D-Max when this axis has a
    /// valid range (`d_range_for_display`), each lowered by `D_STEP_OPTIONAL`. `None` when
    /// no D is known or when applying the hint would switch dynamic D off: a lowered end of
    /// 0, or a lowered base D that reaches the other end of an active range.
    pub fn near_optimal_d_hint(&self, dmax_enabled: bool) -> Option<String> {
        let step = crate::constants::D_STEP_OPTIONAL;
        let lowered_d = self.d.map(|v| v.saturating_sub(step)).filter(|&v| v > 0)?;
        let Some((min, max)) = self.d_range_for_display(dmax_enabled) else {
            return format_d_target(Some(lowered_d), None, None, false)
                .map(|text| format!("{text} [optional D−1]"));
        };
        let (low_min, low_max) = (min.saturating_sub(step), max.saturating_sub(step));
        let text = if low_min > 0 {
            format_d_target(Some(lowered_d), Some(low_min), Some(low_max), true)
        } else if self.d == Some(max) && min < max && lowered_d <= min {
            // Lowering only the upper end would collapse the range (D reaches D-Min).
            return None;
        } else {
            format_d_target(Some(lowered_d), None, None, false)
        };
        text.map(|t| format!("{t} [optional D−1]"))
    }

    /// Legend and console line for a near-optimal axis: the D-1 hint as a conservative
    /// recommendation, or the "none" line when no hint applies.
    pub fn near_optimal_recommendation_line(&self, dmax_enabled: bool) -> String {
        match self.near_optimal_d_hint(dmax_enabled) {
            Some(hint) => format!("Recommendation (conservative): {hint}"),
            None => "Recommendation (none): No obvious tuning adjustments needed".to_string(),
        }
    }

    /// D-Min and D-Max for display and recommendation text: both known, `0 < D-Min <= D-Max`,
    /// on a log with dynamic D enabled; `None` otherwise (callers then show base D). Every
    /// plot, report and console path uses this one rule so they agree per axis.
    pub fn d_range_for_display(&self, dmax_enabled: bool) -> Option<(u32, u32)> {
        match (dmax_enabled, self.d_min, self.d_max) {
            (true, Some(min), Some(max)) if min > 0 && min <= max => Some((min, max)),
            _ => None,
        }
    }

    /// Format every known gain for the report: P, I, D, D-Min, D-Max, FF.
    /// When dynamic D is configured on the log (`dmax_enabled`), every axis with a valid
    /// range (`d_range_for_display`) prints `D-Min:a D-Max:b` in place of the base
    /// D, even when the two are equal, because the base D equals one end of the range and
    /// its meaning changed between Betaflight versions. Otherwise it prints `D:x`. Each
    /// field prints only when the log header carries it. EmuFlight labels feedforward "DF" and hides a zero value;
    /// other firmware prints FF:0.
    pub fn format_for_report(&self, firmware_type: &FirmwareType, dmax_enabled: bool) -> String {
        let mut parts = Vec::new();
        if let Some(p) = self.p {
            parts.push(format!("P:{p}"));
        }
        if let Some(i) = self.i {
            parts.push(format!("I:{i}"));
        }
        if let Some((min, max)) = self.d_range_for_display(dmax_enabled) {
            parts.push(format!("D-Min:{min}"));
            parts.push(format!("D-Max:{max}"));
        } else if let Some(d) = self.d {
            parts.push(format!("D:{d}"));
        }
        match (firmware_type, self.ff) {
            (FirmwareType::EmuFlight, Some(ff)) if ff > 0 => parts.push(format!("DF:{ff}")),
            (FirmwareType::EmuFlight, _) => {}
            (_, Some(ff)) => parts.push(format!("FF:{ff}")),
            _ => {}
        }
        parts.join(" ")
    }

    /// Format PID values for display with firmware-specific terminology
    /// dmax_enabled: Set to true only if D-Min/D-Max system is actively enabled
    pub fn format_for_title(&self, firmware_type: &FirmwareType, dmax_enabled: bool) -> String {
        self.format_for_title_with_ratio(firmware_type, dmax_enabled, false)
    }

    /// Format PID values for display with firmware-specific terminology and optional P:D ratio
    /// dmax_enabled: Set to true only if D-Min/D-Max system is actively enabled
    /// If show_pd_ratio is true and ratio can be calculated, appends P:D ratio
    pub fn format_for_title_with_ratio(
        &self,
        firmware_type: &FirmwareType,
        dmax_enabled: bool,
        show_pd_ratio: bool,
    ) -> String {
        let mut parts = Vec::new();

        if let Some(p) = self.p {
            parts.push(format!("P:{p}"));
        }
        if let Some(i) = self.i {
            parts.push(format!("I:{i}"));
        }

        // D, or D-Min/D-Max when this axis has a valid range on a dynamic-D log
        if let Some((min, max)) = self.d_range_for_display(dmax_enabled) {
            parts.push(format!("D:{min}/{max}"));
        } else if let Some(d) = self.d {
            parts.push(format!("D:{d}"));
        }

        // Add FF before P:D ratio
        // EmuFlight (DF) hides a zero value; other firmware shows an explicit FF:0
        // when the log header carries an FF value of 0.
        if let Some(ff) = self.ff {
            match firmware_type {
                FirmwareType::EmuFlight => {
                    if ff > 0 {
                        parts.push(format!("DF:{ff}"));
                    }
                }
                _ => parts.push(format!("FF:{ff}")),
            }
        }

        // Add P:D ratio if requested and calculable (separated by hyphen for readability)
        if show_pd_ratio {
            if let Some(pd_ratio) = self.calculate_pd_ratio() {
                if parts.is_empty() {
                    return format!(" - P:D={pd_ratio:.2}");
                } else {
                    return format!(" - {} - P:D={pd_ratio:.2}", parts.join(" "));
                }
            }
        }

        if parts.is_empty() {
            String::new()
        } else {
            format!(" - {}", parts.join(" "))
        }
    }
}

/// PID metadata for all three axes (Roll, Pitch, Yaw) with firmware type
#[derive(Debug, Clone, Default)]
pub struct PidMetadata {
    pub roll: AxisPid,  // Axis 0
    pub pitch: AxisPid, // Axis 1
    pub yaw: AxisPid,   // Axis 2
    pub firmware_type: FirmwareType,
    // D-Min/D-Max control parameters - used to determine if dynamic D is enabled
    pub d_max_gain: Option<u32>, // BF 4.5+: Controls gyro-based D boost sensitivity
    pub d_max_advance: Option<u32>, // BF 4.5+: Controls setpoint-based D boost sensitivity
    pub simplified_d_max_gain: Option<u32>, // BF 4.6+: Simplified tuning D-Max gain
}

impl PidMetadata {
    /// Get PID data for a specific axis (ROLL_AXIS=0, PITCH_AXIS=1, YAW_AXIS=2)
    /// Returns None if axis_index is invalid
    #[allow(dead_code)]
    pub fn get_axis(&self, axis_index: usize) -> Option<&AxisPid> {
        match axis_index {
            ROLL_AXIS => Some(&self.roll),
            PITCH_AXIS => Some(&self.pitch),
            YAW_AXIS => Some(&self.yaw),
            _ => None,
        }
    }

    /// Get firmware type
    #[allow(dead_code)]
    pub fn get_firmware_type(&self) -> &FirmwareType {
        &self.firmware_type
    }

    /// Check if D-Min/D-Max dynamic D is configured on any axis.
    ///
    /// The firmware activates dynamic D from the D-Min/D-Max range alone (see
    /// `AxisPid::has_dynamic_d_range`). The gain and advance settings only shape the
    /// boost, and simplified tuning only changes how the stored values are computed.
    /// None of them decide whether the range applies, so none of them gate this check.
    pub fn is_dmax_enabled(&self) -> bool {
        [&self.roll, &self.pitch, &self.yaw]
            .iter()
            .any(|axis| axis.has_dynamic_d_range())
    }
}

/// Format a recommended D target for a legend or console line.
/// With dynamic D enabled and at least one end known, prints `D-Min≈a, D-Max≈b` (a missing
/// end prints `N/A`); otherwise prints the base D as `D≈x`. Returns `None` when no D is known.
pub fn format_d_target(
    d: Option<u32>,
    d_min: Option<u32>,
    d_max: Option<u32>,
    dmax_enabled: bool,
) -> Option<String> {
    if dmax_enabled && (d_min.is_some() || d_max.is_some()) {
        let min = d_min.map_or("N/A".to_string(), |v| v.to_string());
        let max = d_max.map_or("N/A".to_string(), |v| v.to_string());
        Some(format!("D-Min≈{min}, D-Max≈{max}"))
    } else {
        d.map(|v| format!("D≈{v}"))
    }
}

/// Detect firmware type from header metadata map
fn detect_firmware_type(header_map: &std::collections::HashMap<String, String>) -> FirmwareType {
    // Check for firmware revision field (primary detection method)
    if let Some(firmware_rev) = header_map.get("firmware revision") {
        let normalized_rev = firmware_rev.trim().to_lowercase();
        if normalized_rev.contains("emuflight") {
            return FirmwareType::EmuFlight;
        }
        if normalized_rev.contains("betaflight") {
            return FirmwareType::Betaflight;
        }
        if normalized_rev.contains("inav") {
            return FirmwareType::Inav;
        }
    }

    // Fallback: Check for firmware type field (secondary, if revision missing)
    if let Some(firmware_type) = header_map.get("firmware type") {
        let normalized_type = firmware_type.trim().to_lowercase();
        if normalized_type.contains("emuflight") {
            return FirmwareType::EmuFlight;
        }
        if normalized_type.contains("betaflight") {
            return FirmwareType::Betaflight;
        }
        if normalized_type.contains("inav") {
            return FirmwareType::Inav;
        }
    }

    // Fallback: Check for firmware-specific fields
    if header_map.contains_key("df_yaw") {
        return FirmwareType::EmuFlight;
    }

    if header_map.contains_key("ff_weight") {
        return FirmwareType::Betaflight;
    }

    FirmwareType::Unknown
}

/// Helper function to parse D-Min or D-Max values for all axes
fn parse_d_values(
    header_map: &std::collections::HashMap<String, String>,
    comma_separated_key: &str,
    individual_keys: [&str; 6], // [roll_key1, roll_key2, pitch_key1, pitch_key2, yaw_key1, yaw_key2]
) -> [Option<u32>; AXIS_COUNT] {
    let mut values = [None; AXIS_COUNT];

    // Try comma-separated format first
    if let Some(value_str) = header_map.get(comma_separated_key) {
        let parsed_values = parse_comma_separated_values(value_str);
        if parsed_values.len() >= AXIS_COUNT {
            values[ROLL_AXIS] = Some(parsed_values[ROLL_AXIS]);
            values[PITCH_AXIS] = Some(parsed_values[PITCH_AXIS]);
            values[YAW_AXIS] = Some(parsed_values[YAW_AXIS]);
            return values;
        }
    }

    // Fallback to individual fields
    for axis in 0..AXIS_COUNT {
        let key1 = individual_keys[axis * 2];
        let key2 = individual_keys[axis * 2 + 1];

        if let Some(value_str) = header_map.get(key1).or_else(|| header_map.get(key2)) {
            if let Ok(value) = value_str.parse::<u32>() {
                values[axis] = Some(value);
            }
        }
    }

    values
}

/// Helper function to parse FF values for all axes
fn parse_ff_values(
    header_map: &std::collections::HashMap<String, String>,
    existing_pids: &(&AxisPid, &AxisPid, &AxisPid),
) -> [Option<u32>; AXIS_COUNT] {
    let mut ff_values = [
        existing_pids.0.ff, // Roll FF from PID string
        existing_pids.1.ff, // Pitch FF from PID string
        existing_pids.2.ff, // Yaw FF from PID string
    ];

    // Betaflight style: ff_weight with roll,pitch,yaw values (overrides PID string FF)
    if let Some(ff_weight_str) = header_map.get("ff_weight") {
        let values = parse_comma_separated_values(ff_weight_str);
        if values.len() >= AXIS_COUNT {
            for (i, &value) in values.iter().enumerate().take(AXIS_COUNT) {
                ff_values[i] = Some(value);
            }
        }
    }

    // Emuflight style: df_yaw for yaw feedforward only (overrides PID string FF for yaw)
    if let Some(df_yaw_str) = header_map.get("df_yaw") {
        if let Ok(df_yaw_val) = df_yaw_str.parse::<u32>() {
            if df_yaw_val > 0 {
                ff_values[YAW_AXIS] = Some(df_yaw_val); // Yaw DF value
            }
        }
    }

    ff_values
}

/// Helper function to parse axis PID values from header map
fn parse_axis_pids(
    header_map: &std::collections::HashMap<String, String>,
) -> (AxisPid, AxisPid, AxisPid) {
    let roll = if let Some(roll_pid_str) = header_map.get("rollpid") {
        parse_axis_pid(roll_pid_str)
    } else {
        AxisPid::default()
    };

    let pitch = if let Some(pitch_pid_str) = header_map.get("pitchpid") {
        parse_axis_pid(pitch_pid_str)
    } else {
        AxisPid::default()
    };

    let yaw = if let Some(yaw_pid_str) = header_map.get("yawpid") {
        parse_axis_pid(yaw_pid_str)
    } else {
        AxisPid::default()
    };

    (roll, pitch, yaw)
}

/// Parse PID metadata from header key-value pairs
/// Supports Betaflight, Emuflight, and INAV formats
/// Returns default/empty values if no metadata is available
pub fn parse_pid_metadata(header_metadata: &[(String, String)]) -> PidMetadata {
    let mut pid_data = PidMetadata::default();

    // If no metadata available, return default (empty) values
    if header_metadata.is_empty() {
        return pid_data;
    }

    // Convert header metadata to a lookup map for easier access (build once, use everywhere)
    let header_map: std::collections::HashMap<String, String> = header_metadata
        .iter()
        .map(|(k, v)| (k.trim().to_lowercase(), v.trim().to_string()))
        .collect();

    // Detect firmware type using the same map
    pid_data.firmware_type = detect_firmware_type(&header_map);

    // Parse axis PID values
    let (roll, pitch, yaw) = parse_axis_pids(&header_map);
    pid_data.roll = roll;
    pid_data.pitch = pitch;
    pid_data.yaw = yaw;

    // Handle FF values based on flight controller type
    let ff_values = parse_ff_values(
        &header_map,
        &(&pid_data.roll, &pid_data.pitch, &pid_data.yaw),
    );
    pid_data.roll.ff = ff_values[ROLL_AXIS];
    pid_data.pitch.ff = ff_values[PITCH_AXIS];
    pid_data.yaw.ff = ff_values[YAW_AXIS];

    // Look for separate D-Min fields in header metadata (Betaflight separate headers)
    // Only override PID string D-Min values if separate fields exist
    let d_min_values = parse_d_values(
        &header_map,
        "d_min",
        [
            "rolldmin",
            "roll_d_min",
            "pitchdmin",
            "pitch_d_min",
            "yawdmin",
            "yaw_d_min",
        ],
    );
    // Apply D-Min values if available
    if let Some(value) = d_min_values[ROLL_AXIS] {
        pid_data.roll.d_min = Some(value);
    }
    if let Some(value) = d_min_values[PITCH_AXIS] {
        pid_data.pitch.d_min = Some(value);
    }
    if let Some(value) = d_min_values[YAW_AXIS] {
        pid_data.yaw.d_min = Some(value);
    }

    // Look for separate D-Max fields in header metadata (newer Betaflight separate headers)
    // Only override PID string D-Max values if separate fields exist
    let d_max_values = parse_d_values(
        &header_map,
        "d_max",
        [
            "rolldmax",
            "roll_d_max",
            "pitchdmax",
            "pitch_d_max",
            "yawdmax",
            "yaw_d_max",
        ],
    );
    // Apply D-Max values if available
    if let Some(value) = d_max_values[ROLL_AXIS] {
        pid_data.roll.d_max = Some(value);
    }
    if let Some(value) = d_max_values[PITCH_AXIS] {
        pid_data.pitch.d_max = Some(value);
    }
    if let Some(value) = d_max_values[YAW_AXIS] {
        pid_data.yaw.d_max = Some(value);
    }

    // Complete the D-Min/D-Max range. Betaflight stores only one end in a separate header;
    // base D (from the PID string) is the other end, and which end changed over time:
    // - `d_min` header only (BF 4.0 to 4.5): base D is the maximum.
    // - `d_max` header only (BF 2025.12 and master): base D is the minimum.
    // A 5-value PID string or a log with both headers already carries both ends.
    let has_d_min_header = d_min_values.iter().any(Option::is_some);
    let has_d_max_header = d_max_values.iter().any(Option::is_some);
    // The firmware ignores a stored end that lies on the wrong side of base D (`d_max <= D`
    // on 2025.12+, `d_min >= D` on 4.0 to 4.5): D stays constant. Clamp that end to D so
    // the pair never reads as an inverted range, for example after float rounding when
    // simplified tuning computes both from the same default.
    for axis in [&mut pid_data.roll, &mut pid_data.pitch, &mut pid_data.yaw] {
        if has_d_max_header && !has_d_min_header {
            if axis.d_min.is_none() {
                axis.d_min = axis.d;
            }
            if let (Some(d), Some(max)) = (axis.d, axis.d_max) {
                axis.d_max = Some(max.max(d));
            }
        }
        if has_d_min_header && !has_d_max_header {
            if axis.d_max.is_none() {
                axis.d_max = axis.d;
            }
            if let (Some(d), Some(min)) = (axis.d, axis.d_min) {
                axis.d_min = Some(min.min(d));
            }
        }
    }

    // D-Max boost parameters. Renamed across versions: `d_min_gain` / `d_min_advance`
    // (BF 4.0 to 4.2), `d_max_gain` / `d_max_advance` (BF 4.3 and later).
    pid_data.d_max_gain = parse_first_u32(&header_map, &["d_max_gain", "d_min_gain"]);
    pid_data.d_max_advance = parse_first_u32(&header_map, &["d_max_advance", "d_min_advance"]);
    // Simplified tuning D-Max gain: `simplified_dmax_gain` (BF 4.3 to 4.5),
    // `simplified_d_max_gain` (BF 2025.12 and master).
    pid_data.simplified_d_max_gain = parse_first_u32(
        &header_map,
        &["simplified_d_max_gain", "simplified_dmax_gain"],
    );

    pid_data
}

/// Parse the first header in `keys` that holds a valid unsigned integer.
fn parse_first_u32(
    header_map: &std::collections::HashMap<String, String>,
    keys: &[&str],
) -> Option<u32> {
    keys.iter()
        .find_map(|k| header_map.get(*k).and_then(|v| v.parse::<u32>().ok()))
}

/// Parse PID values from a string like "31,56,21" (basic) or "45,80,40,120" (INAV with FF)
/// or "57,66,58,58,206" (Betaflight 4.6+ with P,I,D,D-Max,FF)
fn parse_axis_pid(pid_str: &str) -> AxisPid {
    // Parse by position so an empty or invalid token cannot shift later values forward.
    let tokens: Vec<Option<u32>> = pid_str
        .split(',')
        .map(|t| t.trim().parse::<u32>().ok())
        .collect();
    let values: Vec<u32> = tokens.iter().flatten().copied().collect();
    // A malformed string keeps its leading P, I and D but yields no FF or D-Max.
    let well_formed = values.len() == tokens.len();

    let mut axis_pid = AxisPid {
        p: tokens.first().copied().flatten(),
        i: tokens.get(1).copied().flatten(),
        d: tokens.get(2).copied().flatten(),
        ..Default::default()
    };
    if !well_formed {
        return axis_pid;
    }

    // Handle different formats based on value count
    match values.len() {
        4 => {
            // INAV style: P,I,D,FF
            axis_pid.ff = Some(values[3]);
        }
        5 => {
            // Betaflight 4.6+ style: P,I,D(base=D-Min),D-Max,FF
            // In BF 4.6+, base D IS the minimum (floor), not the maximum
            axis_pid.d_min = Some(values[2]); // Base D is D-Min in BF 4.6+
            axis_pid.d_max = Some(values[3]);
            axis_pid.ff = Some(values[4]);
        }
        _ => {
            // 3 or fewer values: basic P,I,D format (no FF or D-Max)
        }
    }

    axis_pid
}

/// Parse comma-separated values from a string
fn parse_comma_separated_values(value_str: &str) -> Vec<u32> {
    value_str
        .split(',')
        .filter_map(|s| s.trim().parse::<u32>().ok())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pd_advice_change_gate() {
        // Low P:D with significant overshoot: the 2% conservative step is below the 5% gate.
        assert!(!pd_advice_passes_change_gate(
            "Significant overshoot",
            0.32,
            0.32 * 0.98
        ));
        // Normal P:D: the same step clears the gate.
        assert!(pd_advice_passes_change_gate(
            "Significant overshoot",
            2.0,
            2.0 * 0.5
        ));
        // Zones with a small delta by design always pass.
        assert!(pd_advice_passes_change_gate("Acceptable", 1.0, 1.0));
        assert!(pd_advice_passes_change_gate("Undershoot", 1.0, 1.0));
        // Just under the threshold does not pass (strict >).
        assert!(!pd_advice_passes_change_gate("Overshoot", 1.0, 1.05 - 1e-9));
    }

    #[test]
    fn test_betaflight_parsing() {
        let metadata = vec![
            ("rollPID".to_string(), "31,56,21".to_string()),
            ("pitchPID".to_string(), "32,58,23".to_string()),
            ("yawPID".to_string(), "31,56,0".to_string()),
            ("ff_weight".to_string(), "84,87,84".to_string()),
        ];

        let pid_data = parse_pid_metadata(&metadata);

        assert_eq!(pid_data.roll.p, Some(31));
        assert_eq!(pid_data.roll.i, Some(56));
        assert_eq!(pid_data.roll.d, Some(21));
        assert_eq!(pid_data.roll.ff, Some(84));

        assert_eq!(pid_data.pitch.p, Some(32));
        assert_eq!(pid_data.pitch.i, Some(58));
        assert_eq!(pid_data.pitch.d, Some(23));
        assert_eq!(pid_data.pitch.ff, Some(87));

        assert_eq!(pid_data.yaw.p, Some(31));
        assert_eq!(pid_data.yaw.i, Some(56));
        assert_eq!(pid_data.yaw.d, Some(0));
        assert_eq!(pid_data.yaw.ff, Some(84));
    }

    #[test]
    fn test_emuflight_parsing() {
        let metadata = vec![
            ("rollPID".to_string(), "52,57,38".to_string()),
            ("pitchPID".to_string(), "62,57,44".to_string()),
            ("yawPID".to_string(), "90,90,7".to_string()),
            ("df_yaw".to_string(), "15".to_string()),
        ];

        let pid_data = parse_pid_metadata(&metadata);

        assert_eq!(pid_data.roll.p, Some(52));
        assert_eq!(pid_data.roll.i, Some(57));
        assert_eq!(pid_data.roll.d, Some(38));
        assert_eq!(pid_data.roll.ff, None);

        assert_eq!(pid_data.pitch.p, Some(62));
        assert_eq!(pid_data.pitch.i, Some(57));
        assert_eq!(pid_data.pitch.d, Some(44));
        assert_eq!(pid_data.pitch.ff, None);

        assert_eq!(pid_data.yaw.p, Some(90));
        assert_eq!(pid_data.yaw.i, Some(90));
        assert_eq!(pid_data.yaw.d, Some(7));
        assert_eq!(pid_data.yaw.ff, Some(15));
    }

    #[test]
    fn test_zero_ff_parsed_and_displayed() {
        // BF 4.6+ 5-value PID string with FF=0, no ff_weight header
        let metadata = vec![
            (
                "Firmware revision".to_string(),
                "Betaflight 4.6.0".to_string(),
            ),
            ("rollPID".to_string(), "57,66,58,58,0".to_string()),
            ("pitchPID".to_string(), "59,69,72,72,0".to_string()),
            ("yawPID".to_string(), "57,66,0,0,0".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(pid_data.roll.ff, Some(0));
        assert!(pid_data
            .roll
            .format_for_title(&pid_data.firmware_type, false)
            .ends_with("FF:0"));

        // ff_weight header with zeros (BF <4.6 style), mixed with a non-zero axis
        let metadata = vec![
            (
                "Firmware revision".to_string(),
                "Betaflight 4.3.0".to_string(),
            ),
            ("rollPID".to_string(), "40,50,30".to_string()),
            ("pitchPID".to_string(), "40,50,30".to_string()),
            ("yawPID".to_string(), "40,50,0".to_string()),
            ("ff_weight".to_string(), "0,0,80".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(pid_data.roll.ff, Some(0));
        assert_eq!(pid_data.yaw.ff, Some(80));
        assert_eq!(
            pid_data
                .roll
                .format_for_title(&pid_data.firmware_type, false),
            " - P:40 I:50 D:30 FF:0"
        );

        // No FF info at all: nothing printed
        let metadata = vec![
            (
                "Firmware revision".to_string(),
                "Betaflight 4.3.0".to_string(),
            ),
            ("rollPID".to_string(), "40,50,30".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(pid_data.roll.ff, None);
        assert_eq!(
            pid_data
                .roll
                .format_for_title(&pid_data.firmware_type, false),
            " - P:40 I:50 D:30"
        );

        // EmuFlight: zero df_yaw stays hidden
        let metadata = vec![
            (
                "Firmware revision".to_string(),
                "EmuFlight 0.4.0".to_string(),
            ),
            ("yawPID".to_string(), "90,90,7".to_string()),
            ("df_yaw".to_string(), "0".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(
            pid_data
                .yaw
                .format_for_title(&pid_data.firmware_type, false),
            " - P:90 I:90 D:7"
        );
    }

    #[test]
    fn test_inav_zero_ff_and_incomplete_field() {
        // INAV blackbox.c writes rollPID/pitchPID/yawPID as "P,I,D,FF" (4 values).
        let metadata = vec![
            ("Firmware revision".to_string(), "INAV 8.0.0".to_string()),
            ("rollPID".to_string(), "40,30,23,0".to_string()),
            ("pitchPID".to_string(), "44,32,25,60".to_string()),
            ("yawPID".to_string(), "45,35,0,0".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(pid_data.roll.ff, Some(0));
        assert_eq!(pid_data.pitch.ff, Some(60));
        assert_eq!(pid_data.yaw.ff, Some(0));
        assert_eq!(
            pid_data
                .roll
                .format_for_title(&pid_data.firmware_type, false),
            " - P:40 I:30 D:23 FF:0"
        );

        // A trailing empty token is not a value: 3 parsed values, FF stays unknown.
        let metadata = vec![
            ("Firmware revision".to_string(), "INAV 8.0.0".to_string()),
            ("rollPID".to_string(), "40,30,23,".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(pid_data.roll.ff, None);
    }

    #[test]
    fn test_ff_weight_overrides_pid_string_including_zero() {
        let metadata = vec![
            (
                "Firmware revision".to_string(),
                "Betaflight 4.6.0".to_string(),
            ),
            ("rollPID".to_string(), "57,66,58,58,120".to_string()),
            ("ff_weight".to_string(), "0,0,0".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        assert_eq!(pid_data.roll.ff, Some(0));
    }

    #[test]
    fn test_format_for_report_lists_all_known_gains() {
        let metadata = vec![
            (
                "Firmware revision".to_string(),
                "Betaflight 4.6.0".to_string(),
            ),
            ("rollPID".to_string(), "57,66,58,80,0".to_string()),
            ("pitchPID".to_string(), "59,69,72".to_string()),
        ];
        let pid_data = parse_pid_metadata(&metadata);
        let fw = &pid_data.firmware_type;
        assert_eq!(
            pid_data
                .roll
                .format_for_report(fw, pid_data.is_dmax_enabled()),
            "P:57 I:66 D-Min:58 D-Max:80 FF:0"
        );
        // Fields absent from the header are not printed.
        assert_eq!(
            pid_data
                .pitch
                .format_for_report(fw, pid_data.is_dmax_enabled()),
            "P:59 I:69 D:72"
        );
        // EmuFlight: DF label, zero hidden.
        let ef = AxisPid {
            p: Some(52),
            i: Some(57),
            d: Some(38),
            ff: Some(0),
            ..Default::default()
        };
        assert_eq!(
            ef.format_for_report(&FirmwareType::EmuFlight, false),
            "P:52 I:57 D:38"
        );
        let ef = AxisPid { ff: Some(15), ..ef };
        assert_eq!(
            ef.format_for_report(&FirmwareType::EmuFlight, false),
            "P:52 I:57 D:38 DF:15"
        );
    }

    #[test]
    fn test_inav_parsing() {
        let metadata = vec![
            ("rollPID".to_string(), "45,80,40,120".to_string()),
            ("pitchPID".to_string(), "47,84,46,125".to_string()),
            ("yawPID".to_string(), "45,80,0,120".to_string()),
        ];

        let pid_data = parse_pid_metadata(&metadata);

        assert_eq!(pid_data.roll.p, Some(45));
        assert_eq!(pid_data.roll.i, Some(80));
        assert_eq!(pid_data.roll.d, Some(40));
        assert_eq!(pid_data.roll.ff, Some(120));

        assert_eq!(pid_data.pitch.p, Some(47));
        assert_eq!(pid_data.pitch.i, Some(84));
        assert_eq!(pid_data.pitch.d, Some(46));
        assert_eq!(pid_data.pitch.ff, Some(125));

        assert_eq!(pid_data.yaw.p, Some(45));
        assert_eq!(pid_data.yaw.i, Some(80));
        assert_eq!(pid_data.yaw.d, Some(0));
        assert_eq!(pid_data.yaw.ff, Some(120));
    }

    #[test]
    fn test_betaflight_5_value_parsing() {
        // Test Betaflight 4.6+ format: P,I,D(base=D-Min),D-Max,FF
        // In BF 4.6+, base D is the minimum value, not the maximum
        let metadata = vec![
            ("rollPID".to_string(), "57,66,58,58,206".to_string()),
            ("pitchPID".to_string(), "59,69,72,72,215".to_string()),
            ("yawPID".to_string(), "57,66,0,0,206".to_string()),
        ];

        let pid_data = parse_pid_metadata(&metadata);

        assert_eq!(pid_data.roll.p, Some(57));
        assert_eq!(pid_data.roll.i, Some(66));
        assert_eq!(pid_data.roll.d, Some(58));
        assert_eq!(pid_data.roll.d_min, Some(58)); // Base D is D-Min in BF 4.6+
        assert_eq!(pid_data.roll.d_max, Some(58));
        assert_eq!(pid_data.roll.ff, Some(206));

        assert_eq!(pid_data.pitch.p, Some(59));
        assert_eq!(pid_data.pitch.i, Some(69));
        assert_eq!(pid_data.pitch.d, Some(72));
        assert_eq!(pid_data.pitch.d_min, Some(72)); // Base D is D-Min in BF 4.6+
        assert_eq!(pid_data.pitch.d_max, Some(72));
        assert_eq!(pid_data.pitch.ff, Some(215));

        assert_eq!(pid_data.yaw.p, Some(57));
        assert_eq!(pid_data.yaw.i, Some(66));
        assert_eq!(pid_data.yaw.d, Some(0));
        assert_eq!(pid_data.yaw.d_min, Some(0)); // Base D is D-Min in BF 4.6+
        assert_eq!(pid_data.yaw.d_max, Some(0));
        assert_eq!(pid_data.yaw.ff, Some(206));
    }

    #[test]
    fn test_betaflight_d_min_d_max_parsing() {
        // Test Betaflight 4.5 format with separate d_min and d_max fields
        let metadata = vec![
            ("rollPID".to_string(), "57,66,58".to_string()),
            ("pitchPID".to_string(), "59,69,72".to_string()),
            ("yawPID".to_string(), "57,66,0".to_string()),
            ("d_min".to_string(), "39,44,0".to_string()),
            ("d_max".to_string(), "80,90,0".to_string()),
            ("ff_weight".to_string(), "206,215,206".to_string()),
        ];

        let pid_data = parse_pid_metadata(&metadata);

        assert_eq!(pid_data.roll.p, Some(57));
        assert_eq!(pid_data.roll.i, Some(66));
        assert_eq!(pid_data.roll.d, Some(58));
        assert_eq!(pid_data.roll.d_min, Some(39));
        assert_eq!(pid_data.roll.d_max, Some(80));
        assert_eq!(pid_data.roll.ff, Some(206));

        assert_eq!(pid_data.pitch.d_min, Some(44));
        assert_eq!(pid_data.pitch.d_max, Some(90));

        assert_eq!(pid_data.yaw.d_min, Some(0));
        assert_eq!(pid_data.yaw.d_max, Some(0));

        // VERIFY D:XX FORMATTING (D-Max disabled since no gain/advance values)
        let roll_formatted = pid_data
            .roll
            .format_for_title(&FirmwareType::Betaflight, false);
        assert_eq!(roll_formatted, " - P:57 I:66 D:58 FF:206");

        let pitch_formatted = pid_data
            .pitch
            .format_for_title(&FirmwareType::Betaflight, false);
        assert_eq!(pitch_formatted, " - P:59 I:69 D:72 FF:215");
    }

    #[test]
    fn test_format_for_title() {
        let axis_pid = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            ff: Some(84),
            ..Default::default()
        };

        // Test Betaflight formatting
        assert_eq!(
            axis_pid.format_for_title(&FirmwareType::Betaflight, false),
            " - P:31 I:56 D:21 FF:84"
        );

        // Test EmuFlight formatting
        assert_eq!(
            axis_pid.format_for_title(&FirmwareType::EmuFlight, false),
            " - P:31 I:56 D:21 DF:84"
        );

        // Zero FF: shown as FF:0 for non-EmuFlight, omitted for EmuFlight (DF)
        let axis_pid_zero_ff = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            ff: Some(0),
            ..Default::default()
        };
        assert_eq!(
            axis_pid_zero_ff.format_for_title(&FirmwareType::Betaflight, false),
            " - P:31 I:56 D:21 FF:0"
        );
        assert_eq!(
            axis_pid_zero_ff.format_for_title(&FirmwareType::EmuFlight, false),
            " - P:31 I:56 D:21"
        );

        // Test with no FF
        let axis_pid_no_ff = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            ff: None,
            ..Default::default()
        };
        assert_eq!(
            axis_pid_no_ff.format_for_title(&FirmwareType::Betaflight, false),
            " - P:31 I:56 D:21"
        );

        // Test D:XX/XX format when D and D-Max are different (with dmax_enabled=true)
        let axis_pid_diff_dmax = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: Some(21),
            d_max: Some(35),
            ff: None,
        };
        assert_eq!(
            axis_pid_diff_dmax.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:21/35"
        );

        // Test D:XX format when D and D-Max are the same
        let axis_pid_same_dmax = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_max: Some(21),
            ff: None,
            ..Default::default()
        };
        assert_eq!(
            axis_pid_same_dmax.format_for_title(&FirmwareType::Betaflight, false),
            " - P:31 I:56 D:21"
        );

        // Test D:min/max format when D-Min and D-Max are different
        let axis_pid_dmin_dmax = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: Some(15),
            d_max: Some(35),
            ff: None,
        };
        assert_eq!(
            axis_pid_dmin_dmax.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:15/35"
        );

        // Test D:min/max format with FF (with dmax_enabled=true)
        let axis_pid_dmin_dmax_ff = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: Some(15),
            d_max: Some(35),
            ff: Some(84),
        };
        assert_eq!(
            axis_pid_dmin_dmax_ff.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:15/35 FF:84"
        );
    }

    #[test]
    fn test_format_for_title_zero_handling() {
        // D-Max 0 is not a range end (firmware keeps D constant): base D, with dmax_enabled=true
        let axis_pid_dmax_zero = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: Some(15),
            d_max: Some(0), // D-Max is zero
            ff: None,
        };
        assert_eq!(
            axis_pid_dmax_zero.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:21"
        );

        // D-Min 0 disables dynamic D on the axis: base D, with dmax_enabled=true
        let axis_pid_dmin_zero = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: Some(0), // D-Min is zero
            d_max: Some(35),
            ff: None,
        };
        assert_eq!(
            axis_pid_dmin_zero.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:21"
        );

        // Test D:XX/0 with only D-Max (no D-Min) should become D:XX - with dmax_enabled=true
        let axis_pid_only_dmax_zero = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: None,
            d_max: Some(0), // D-Max is zero
            ff: None,
        };
        assert_eq!(
            axis_pid_only_dmax_zero.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:21"
        );

        // Test normal case still works (both non-zero and different) - with dmax_enabled=true
        let axis_pid_normal = AxisPid {
            p: Some(31),
            i: Some(56),
            d: Some(21),
            d_min: Some(15),
            d_max: Some(35),
            ff: None,
        };
        assert_eq!(
            axis_pid_normal.format_for_title(&FirmwareType::Betaflight, true),
            " - P:31 I:56 D:15/35"
        );
    }

    #[test]
    fn test_empty_metadata() {
        let empty_metadata = vec![];
        let pid_data = parse_pid_metadata(&empty_metadata);

        // Should return default values
        assert_eq!(pid_data.roll.p, None);
        assert_eq!(pid_data.roll.i, None);
        assert_eq!(pid_data.roll.d, None);
        assert_eq!(pid_data.roll.ff, None);

        // Title should be empty
        assert_eq!(
            pid_data
                .roll
                .format_for_title(&FirmwareType::Unknown, false),
            ""
        );
    }

    #[test]
    fn test_firmware_detection_case_insensitive() {
        // Test case-insensitive firmware detection with various whitespace and case combinations

        // Betaflight variations
        let metadata_bf1 = vec![(
            "firmware revision".to_string(),
            "BETAFLIGHT 4.6.0".to_string(),
        )];
        let pid_data_bf1 = parse_pid_metadata(&metadata_bf1);
        assert_eq!(pid_data_bf1.firmware_type, FirmwareType::Betaflight);

        let metadata_bf2 = vec![(
            "firmware revision".to_string(),
            "  betaflight 4.5.1  ".to_string(),
        )];
        let pid_data_bf2 = parse_pid_metadata(&metadata_bf2);
        assert_eq!(pid_data_bf2.firmware_type, FirmwareType::Betaflight);

        let metadata_bf3 = vec![(
            "firmware revision".to_string(),
            "BetaFlight Custom Build".to_string(),
        )];
        let pid_data_bf3 = parse_pid_metadata(&metadata_bf3);
        assert_eq!(pid_data_bf3.firmware_type, FirmwareType::Betaflight);

        // EmuFlight variations
        let metadata_ef1 = vec![(
            "firmware revision".to_string(),
            "EMUFLIGHT 0.4.2".to_string(),
        )];
        let pid_data_ef1 = parse_pid_metadata(&metadata_ef1);
        assert_eq!(pid_data_ef1.firmware_type, FirmwareType::EmuFlight);

        let metadata_ef2 = vec![(
            "firmware revision".to_string(),
            "  EmuFlight Beta  ".to_string(),
        )];
        let pid_data_ef2 = parse_pid_metadata(&metadata_ef2);
        assert_eq!(pid_data_ef2.firmware_type, FirmwareType::EmuFlight);

        // INAV variations
        let metadata_inav1 = vec![("firmware revision".to_string(), "INAV 8.0.0".to_string())];
        let pid_data_inav1 = parse_pid_metadata(&metadata_inav1);
        assert_eq!(pid_data_inav1.firmware_type, FirmwareType::Inav);

        let metadata_inav2 = vec![(
            "firmware revision".to_string(),
            "  iNav Latest  ".to_string(),
        )];
        let pid_data_inav2 = parse_pid_metadata(&metadata_inav2);
        assert_eq!(pid_data_inav2.firmware_type, FirmwareType::Inav);

        // Fallback to firmware type field
        let metadata_type = vec![("firmware type".to_string(), "  BETAFLIGHT  ".to_string())];
        let pid_data_type = parse_pid_metadata(&metadata_type);
        assert_eq!(pid_data_type.firmware_type, FirmwareType::Betaflight);

        // Field-based detection (when text-based fails)
        let metadata_field = vec![("df_yaw".to_string(), "15".to_string())];
        let pid_data_field = parse_pid_metadata(&metadata_field);
        assert_eq!(pid_data_field.firmware_type, FirmwareType::EmuFlight);
    }

    #[test]
    fn test_whitespace_trimming() {
        // Test that whitespace in keys and values is properly trimmed
        let metadata = vec![
            ("  rollPID  ".to_string(), "  31,56,21  ".to_string()),
            (
                " firmware revision ".to_string(),
                " Betaflight 4.6.0 ".to_string(),
            ),
            ("ff_weight".to_string(), " 84,87,84 ".to_string()),
        ];

        let pid_data = parse_pid_metadata(&metadata);

        // Should correctly parse despite whitespace
        assert_eq!(pid_data.firmware_type, FirmwareType::Betaflight);
        assert_eq!(pid_data.roll.p, Some(31));
        assert_eq!(pid_data.roll.i, Some(56));
        assert_eq!(pid_data.roll.d, Some(21));
        assert_eq!(pid_data.roll.ff, Some(84));
    }

    #[test]
    fn test_pd_ratio_calculation() {
        // Test normal P:D ratio calculation
        let axis_pid = AxisPid {
            p: Some(54),
            i: Some(60),
            d: Some(32),
            d_min: None,
            d_max: None,
            ff: None,
        };
        let ratio = axis_pid.calculate_pd_ratio().unwrap();
        assert!((ratio - 1.6875).abs() < 0.001); // 54/32 = 1.6875

        // Test with D-Min/D-Max - should still use base D
        let axis_pid_dminmax = AxisPid {
            p: Some(60),
            i: Some(65),
            d: Some(40),
            d_min: Some(36),
            d_max: Some(45),
            ff: None,
        };
        let ratio_dminmax = axis_pid_dminmax.calculate_pd_ratio().unwrap();
        assert!((ratio_dminmax - 1.5).abs() < 0.001); // 60/40 = 1.5 (uses base d)

        // Test edge case: P is 0 (should still calculate, though unusual)
        let axis_pid_p_zero = AxisPid {
            p: Some(0),
            i: Some(50),
            d: Some(30),
            d_min: None,
            d_max: None,
            ff: None,
        };
        let ratio_p_zero = axis_pid_p_zero.calculate_pd_ratio().unwrap();
        assert_eq!(ratio_p_zero, 0.0);
    }

    #[test]
    fn test_pd_ratio_none_cases() {
        // Test when P is missing
        let axis_pid_no_p = AxisPid {
            p: None,
            i: Some(60),
            d: Some(32),
            d_min: None,
            d_max: None,
            ff: None,
        };
        assert_eq!(axis_pid_no_p.calculate_pd_ratio(), None);

        // Test when D is missing
        let axis_pid_no_d = AxisPid {
            p: Some(54),
            i: Some(60),
            d: None,
            d_min: None,
            d_max: None,
            ff: None,
        };
        assert_eq!(axis_pid_no_d.calculate_pd_ratio(), None);

        // Test when D is 0
        let axis_pid_d_zero = AxisPid {
            p: Some(54),
            i: Some(60),
            d: Some(0),
            d_min: None,
            d_max: None,
            ff: None,
        };
        assert_eq!(axis_pid_d_zero.calculate_pd_ratio(), None);

        // Test when D-Min is 0 but D is valid (should use D)
        let axis_pid_dmin_zero = AxisPid {
            p: Some(54),
            i: Some(60),
            d: Some(32),
            d_min: Some(0),
            d_max: None,
            ff: None,
        };
        let ratio = axis_pid_dmin_zero.calculate_pd_ratio().unwrap();
        assert!((ratio - 1.6875).abs() < 0.001); // Falls back to d=32
    }

    #[test]
    fn test_goal_d_calculation() {
        // Test calculation for target ratio of 1.4
        let axis_pid = AxisPid {
            p: Some(54),
            i: Some(60),
            d: Some(32),
            d_min: None,
            d_max: None,
            ff: None,
        };
        let goal_d = axis_pid.calculate_goal_d_for_ratio(1.4).unwrap();
        assert_eq!(goal_d, 39); // 54 / 1.4 = 38.57, rounds to 39

        // Test with different P value
        let axis_pid2 = AxisPid {
            p: Some(60),
            i: Some(65),
            d: Some(36),
            d_min: None,
            d_max: None,
            ff: None,
        };
        let goal_d2 = axis_pid2.calculate_goal_d_for_ratio(1.4).unwrap();
        assert_eq!(goal_d2, 43); // 60 / 1.4 = 42.86, rounds to 43

        // Test when P is None
        let axis_pid_no_p = AxisPid {
            p: None,
            i: Some(60),
            d: Some(32),
            d_min: None,
            d_max: None,
            ff: None,
        };
        assert_eq!(axis_pid_no_p.calculate_goal_d_for_ratio(1.4), None);

        // Test with invalid target ratio
        assert_eq!(axis_pid.calculate_goal_d_for_ratio(0.0), None);
        assert_eq!(axis_pid.calculate_goal_d_for_ratio(-1.0), None);
    }

    #[test]
    fn test_format_with_pd_ratio() {
        // Test formatting with P:D ratio included (separated by hyphen)
        let axis_pid = AxisPid {
            p: Some(54),
            i: Some(60),
            d: Some(32),
            d_min: None,
            d_max: None,
            ff: Some(100),
        };

        // Without ratio
        let formatted_no_ratio = axis_pid.format_for_title(&FirmwareType::Betaflight, false);
        assert_eq!(formatted_no_ratio, " - P:54 I:60 D:32 FF:100");

        // With ratio (note: hyphen separator before P:D ratio)
        let formatted_with_ratio =
            axis_pid.format_for_title_with_ratio(&FirmwareType::Betaflight, false, true);
        assert_eq!(formatted_with_ratio, " - P:54 I:60 D:32 FF:100 - P:D=1.69");

        // Test with D-Min/D-Max and ratio (hyphen separator)
        let axis_pid_dmin = AxisPid {
            p: Some(60),
            i: Some(65),
            d: Some(40),
            d_min: Some(36),
            d_max: Some(45),
            ff: None,
        };
        let formatted_dmin_ratio =
            axis_pid_dmin.format_for_title_with_ratio(&FirmwareType::Betaflight, true, true);
        assert_eq!(formatted_dmin_ratio, " - P:60 I:65 D:36/45 - P:D=1.50");

        // Test when ratio can't be calculated (no hyphen separator needed)
        let axis_pid_no_d = AxisPid {
            p: Some(54),
            i: Some(60),
            d: None,
            d_min: None,
            d_max: None,
            ff: None,
        };
        let formatted_no_d =
            axis_pid_no_d.format_for_title_with_ratio(&FirmwareType::Betaflight, false, true);
        assert_eq!(formatted_no_d, " - P:54 I:60"); // No P:D ratio shown
    }

    #[test]
    fn test_format_d_term_edge_cases() {
        // Both ends known and ordered: range.
        let axis_range = AxisPid {
            d: Some(50),
            d_min: Some(30),
            d_max: Some(50),
            ..Default::default()
        };
        assert!(axis_range
            .format_for_title(&FirmwareType::Betaflight, true)
            .contains("D:30/50"));

        // One end missing or the pair inverted: base D, never a partial or inverted range.
        for (d_min, d_max) in [(Some(30), None), (None, Some(80)), (Some(60), Some(40))] {
            let axis = AxisPid {
                d: Some(50),
                d_min,
                d_max,
                ..Default::default()
            };
            let title = axis.format_for_title(&FirmwareType::Betaflight, true);
            assert!(title.contains("D:50") && !title.contains('/'), "{title}");
            assert_eq!(axis.d_range_for_display(true), None);
        }
    }

    fn headers(rev: &str, extra: &[(&str, &str)]) -> Vec<(String, String)> {
        let mut v = vec![("Firmware revision".to_string(), rev.to_string())];
        v.extend(extra.iter().map(|(k, x)| (k.to_string(), x.to_string())));
        v
    }

    #[test]
    fn test_dynamic_d_legacy_layout_bf_4_0_to_4_5() {
        // Values from real BF 4.0.2 (d_min_gain names) and 4.5.0 (d_max_gain names) logs.
        for (rev, gain, adv) in [
            (
                "Betaflight 4.0.2 (56bdc8d26) AIKONF4",
                "d_min_gain",
                "d_min_advance",
            ),
            (
                "Betaflight 4.5.0 (f69fedd1f) STM32F7X2",
                "d_max_gain",
                "d_max_advance",
            ),
        ] {
            let pid = parse_pid_metadata(&headers(
                rev,
                &[
                    ("rollPID", "31,65,21"),
                    ("pitchPID", "37,75,23"),
                    ("yawPID", "45,120,0"),
                    ("d_min", "20,22,0"),
                    (gain, "27"),
                    (adv, "20"),
                ],
            ));
            assert!(pid.is_dmax_enabled(), "{rev}");
            assert_eq!(pid.d_max_gain, Some(27), "{rev}");
            assert_eq!(pid.d_max_advance, Some(20), "{rev}");
            // D (21) is the maximum; D-Min comes from the d_min header.
            assert_eq!(
                pid.roll
                    .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
                "P:31 I:65 D-Min:20 D-Max:21"
            );
            assert_eq!(
                pid.pitch
                    .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
                "P:37 I:75 D-Min:22 D-Max:23"
            );
            // Yaw: d_min 0 means no dynamic D.
            assert_eq!(
                pid.yaw
                    .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
                "P:45 I:120 D:0"
            );
            assert_eq!(
                pid.roll
                    .format_for_title(&pid.firmware_type, pid.is_dmax_enabled()),
                " - P:31 I:65 D:20/21"
            );
        }
    }

    #[test]
    fn test_dynamic_d_new_layout_bf_2025_12_and_master() {
        // BF 2025.12+ blackbox.c: rollPID is "P,I,D" with D the minimum, plus a d_max header.
        // Simplified tuning off and on give the same result: the range alone decides.
        for simplified in [
            ("simplified_pids_mode", "0"),
            ("simplified_d_max_gain", "100"),
        ] {
            let pid = parse_pid_metadata(&headers(
                "Betaflight 2025.12.0 (abc) STM32H743",
                &[
                    ("rollPID", "38,81,21"),
                    ("pitchPID", "40,83,23"),
                    ("yawPID", "30,80,0"),
                    ("d_max", "35,38,0"),
                    ("d_max_gain", "37"),
                    ("d_max_advance", "20"),
                    simplified,
                ],
            ));
            assert!(pid.is_dmax_enabled());
            assert_eq!(
                pid.roll
                    .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
                "P:38 I:81 D-Min:21 D-Max:35"
            );
            assert_eq!(
                pid.pitch
                    .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
                "P:40 I:83 D-Min:23 D-Max:38"
            );
            assert_eq!(
                pid.yaw
                    .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
                "P:30 I:80 D:0"
            );
            assert_eq!(
                pid.roll
                    .format_for_title(&pid.firmware_type, pid.is_dmax_enabled()),
                " - P:38 I:81 D:21/35"
            );
        }
        // Header name for the simplified gain differs from 4.3 to 4.5.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2025.12.0",
            &[("simplified_d_max_gain", "100")],
        ));
        assert_eq!(pid.simplified_d_max_gain, Some(100));
    }

    #[test]
    fn test_dynamic_d_every_axis_prints_both_ends_when_enabled() {
        // Real BF 2026.12.0 log (LIONBEE): roll has a range (80 to 82), pitch has
        // d_max equal to D (106), yaw has D 0 and d_max 0. Dynamic D is enabled by roll,
        // so every axis with both ends known prints them in the same form.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2026.12.0 (11b0f998f) AT32F435G",
            &[
                ("rollPID", "47,76,80"),
                ("pitchPID", "67,96,106"),
                ("yawPID", "35,65,0"),
                ("d_max", "82,106,0"),
                ("d_max_gain", "0"),
                ("d_max_advance", "35"),
            ],
        ));
        assert!(pid.is_dmax_enabled());
        let fw = &pid.firmware_type;
        assert_eq!(
            pid.roll.format_for_report(fw, true),
            "P:47 I:76 D-Min:80 D-Max:82"
        );
        assert_eq!(
            pid.pitch.format_for_report(fw, true),
            "P:67 I:96 D-Min:106 D-Max:106"
        );
        assert_eq!(pid.yaw.format_for_report(fw, true), "P:35 I:65 D:0");
        assert_eq!(pid.roll.format_for_title(fw, true), " - P:47 I:76 D:80/82");
        assert_eq!(
            pid.pitch.format_for_title(fw, true),
            " - P:67 I:96 D:106/106"
        );
        assert_eq!(pid.yaw.format_for_title(fw, true), " - P:35 I:65 D:0");
    }

    #[test]
    fn test_format_d_target() {
        // Dynamic D on: both ends, a missing end prints N/A.
        assert_eq!(
            format_d_target(Some(108), Some(108), Some(108), true).as_deref(),
            Some("D-Min≈108, D-Max≈108")
        );
        assert_eq!(
            format_d_target(Some(83), Some(83), None, true).as_deref(),
            Some("D-Min≈83, D-Max≈N/A")
        );
        // Dynamic D off, or no ends known on this axis: base D.
        assert_eq!(
            format_d_target(Some(83), Some(80), Some(82), false).as_deref(),
            Some("D≈83")
        );
        assert_eq!(
            format_d_target(Some(0), None, None, true).as_deref(),
            Some("D≈0")
        );
        assert_eq!(format_d_target(None, None, None, true), None);
    }

    #[test]
    fn test_dynamic_d_simplified_tuning_slider_off() {
        // BF 4.5 simplified_tuning.c: with simplified_dmin_ratio 0, dminRatio is 1.0, so
        // D and d_min come from the same expression and are equal. No range, single D.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[
                ("rollPID", "47,76,40"),
                ("pitchPID", "55,80,46"),
                ("yawPID", "45,80,0"),
                ("d_min", "40,46,0"),
                ("d_max_gain", "37"),
                ("d_max_advance", "20"),
                ("simplified_pids_mode", "2"),
                ("simplified_dmax_gain", "0"),
            ],
        ));
        assert!(!pid.is_dmax_enabled());
        let fw = &pid.firmware_type;
        assert_eq!(
            pid.roll.format_for_report(fw, pid.is_dmax_enabled()),
            "P:47 I:76 D:40"
        );
        assert_eq!(
            pid.roll.format_for_title(fw, pid.is_dmax_enabled()),
            " - P:47 I:76 D:40"
        );

        // BF 2025.12 simplified_tuning.c: with simplified_d_max_gain 0, d_max equals D.
        // Real log values (LIONBEE flight 15, simplified_pids_mode 2).
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2026.12.0-alpha (29ae6a65e) AT32F435G",
            &[
                ("rollPID", "45,80,30"),
                ("pitchPID", "47,84,34"),
                ("yawPID", "45,80,0"),
                ("d_max", "30,34,0"),
                ("d_max_gain", "0"),
                ("d_max_advance", "35"),
                ("simplified_pids_mode", "2"),
                ("simplified_d_max_gain", "0"),
            ],
        ));
        assert!(!pid.is_dmax_enabled());
        let fw = &pid.firmware_type;
        assert_eq!(pid.roll.format_for_report(fw, false), "P:45 I:80 D:30");
        assert_eq!(pid.pitch.format_for_report(fw, false), "P:47 I:84 D:34");
        assert_eq!(pid.yaw.format_for_report(fw, false), "P:45 I:80 D:0");
    }

    #[test]
    fn test_dynamic_d_end_on_wrong_side_of_base_d_is_clamped() {
        // 2025.12+: d_max below D (float rounding) means no boost; the pair must not invert.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2025.12.0",
            &[
                ("rollPID", "47,76,80"),
                ("pitchPID", "67,96,106"),
                ("d_max", "90,105,0"),
            ],
        ));
        assert_eq!((pid.pitch.d_min, pid.pitch.d_max), (Some(106), Some(106)));
        assert!(pid.is_dmax_enabled()); // roll 80 to 90
        assert_eq!(
            pid.pitch.format_for_report(&pid.firmware_type, true),
            "P:67 I:96 D-Min:106 D-Max:106"
        );
        // 4.0 to 4.5: d_min above D means the firmware does not use dynamic D on that axis.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.4.0",
            &[("rollPID", "51,55,47"), ("d_min", "50,0,0")],
        ));
        assert_eq!((pid.roll.d_min, pid.roll.d_max), (Some(47), Some(47)));
        assert!(!pid.is_dmax_enabled());
    }

    #[test]
    fn test_dynamic_d_mixed_axes_follow_per_axis_rule() {
        // BF 4.5: roll has a range (20 to 30); pitch has d_min 0, so the firmware keeps a
        // single D on pitch. Title, report and recommendations must all show base D there.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[
                ("rollPID", "40,50,30"),
                ("pitchPID", "42,52,40"),
                ("yawPID", "45,80,0"),
                ("d_min", "20,0,0"),
            ],
        ));
        let on = pid.is_dmax_enabled();
        assert!(on);
        let fw = &pid.firmware_type;
        assert_eq!(pid.roll.format_for_title(fw, on), " - P:40 I:50 D:20/30");
        assert_eq!(pid.pitch.format_for_title(fw, on), " - P:42 I:52 D:40");
        assert_eq!(
            pid.roll.format_for_report(fw, on),
            "P:40 I:50 D-Min:20 D-Max:30"
        );
        assert_eq!(pid.pitch.format_for_report(fw, on), "P:42 I:52 D:40");
        // Recommendation for pitch target P:D 1.0 -> D 42: single D, no D-Min 0.
        assert_eq!(
            pid.pitch.calculate_goal_d_with_range(1.0, on),
            (Some(42), None, None)
        );
        // Roll scales both ends by the same factor as base D (30 -> 42 is x1.4).
        assert_eq!(
            pid.roll.calculate_goal_d_with_range(40.0 / 42.0, on),
            (Some(42), Some(28), Some(42))
        );
    }

    #[test]
    fn test_dynamic_d_title_and_report_agree_when_base_d_is_zero() {
        // BF master: pitch D is 0 with d_max above it. The firmware needs D > 0, so no range.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2026.12.0",
            &[
                ("rollPID", "40,40,20"),
                ("pitchPID", "40,40,0"),
                ("d_max", "30,40,0"),
            ],
        ));
        let on = pid.is_dmax_enabled();
        assert!(on); // roll
        let fw = &pid.firmware_type;
        assert_eq!(pid.pitch.format_for_title(fw, on), " - P:40 I:40 D:0");
        assert_eq!(pid.pitch.format_for_report(fw, on), "P:40 I:40 D:0");
    }

    #[test]
    fn test_dynamic_d_on_yaw_only_enables_the_log_flag() {
        // BF 4.5 applies the activation rule to yaw as well (d_min_yaw).
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[
                ("rollPID", "40,50,30"),
                ("pitchPID", "42,52,40"),
                ("yawPID", "45,80,25"),
                ("d_min", "0,0,15"),
            ],
        ));
        assert!(pid.is_dmax_enabled());
        let fw = &pid.firmware_type;
        assert_eq!(
            pid.yaw.format_for_report(fw, true),
            "P:45 I:80 D-Min:15 D-Max:25"
        );
        assert_eq!(pid.roll.format_for_report(fw, true), "P:40 I:50 D:30");
    }

    #[test]
    fn test_inverted_pair_from_unexpected_headers_shows_base_d() {
        // Both range headers present: layout is ambiguous, so no end is rewritten. An
        // inverted pair must not print as a range.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[
                ("rollPID", "40,50,30"),
                ("d_min", "50,0,0"),
                ("d_max", "35,0,0"),
            ],
        ));
        assert_eq!(
            pid.roll.format_for_report(&pid.firmware_type, true),
            "P:40 I:50 D:30"
        );
        assert_eq!(pid.roll.d_range_for_display(true), None);
    }

    #[test]
    fn test_malformed_pid_string_gives_no_ff_or_range() {
        // A trailing empty token is a missing value, not a 4-value string with FF 0.
        let pid = parse_pid_metadata(&headers("Betaflight 4.6.0", &[("rollPID", "57,66,58,0,")]));
        assert_eq!(
            (pid.roll.p, pid.roll.i, pid.roll.d),
            (Some(57), Some(66), Some(58))
        );
        assert_eq!(
            (pid.roll.ff, pid.roll.d_min, pid.roll.d_max),
            (None, None, None)
        );
        // An invalid token in the middle does not shift later values into earlier slots.
        let pid = parse_pid_metadata(&headers("Betaflight 4.4.0", &[("rollPID", "57,x,58")]));
        assert_eq!(
            (pid.roll.p, pid.roll.i, pid.roll.d),
            (Some(57), None, Some(58))
        );
    }

    #[test]
    fn test_aggressive_recommendation_is_the_one_shared_source() {
        // BF 4.5: roll has a range (20 to 30), pitch has d_min 0 and shows a single D.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[
                ("rollPID", "40,50,30"),
                ("pitchPID", "42,52,40"),
                ("d_min", "20,0,0"),
            ],
        ));
        let on = pid.is_dmax_enabled();
        // Current P:D 1.0 times PD_RATIO_AGGRESSIVE_MULTIPLIER (0.65): P 40 needs D 40 / 0.65 = 62.
        let roll = pid.roll.aggressive_recommendation(1.0, on);
        assert!((roll.pd_ratio - 0.65).abs() < 1e-12);
        assert_eq!(roll.d, Some(62));
        assert_eq!((roll.d_min, roll.d_max), (Some(41), Some(62)));
        // Same call on pitch (P 42): single D, no D-Min or D-Max.
        let pitch = pid.pitch.aggressive_recommendation(1.0, on);
        assert_eq!(pitch.d, Some(65));
        assert_eq!((pitch.d_min, pitch.d_max), (None, None));
        // It matches calculate_goal_d_with_range at the same ratio.
        assert_eq!(
            (roll.d, roll.d_min, roll.d_max),
            pid.roll.calculate_goal_d_with_range(0.65, on)
        );
    }

    #[test]
    fn test_near_optimal_d_hint_follows_the_per_axis_rule() {
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[
                ("rollPID", "40,50,30"),
                ("pitchPID", "42,52,40"),
                ("d_min", "20,0,0"),
            ],
        ));
        let on = pid.is_dmax_enabled();
        // Roll has a range: both ends lowered by D_STEP_OPTIONAL (1).
        assert_eq!(
            pid.roll.near_optimal_d_hint(on).as_deref(),
            Some("D-Min≈19, D-Max≈29 [optional D−1]")
        );
        // Pitch has d_min 0: single D lowered by 1, same as a log without dynamic D.
        assert_eq!(
            pid.pitch.near_optimal_d_hint(on).as_deref(),
            Some("D≈39 [optional D−1]")
        );
        assert_eq!(AxisPid::default().near_optimal_d_hint(on), None);
    }

    #[test]
    fn test_near_optimal_d_hint_never_lowers_a_range_end_to_zero() {
        // BF 4.5 range D-Min 1 to D 20: D-Min 1 lowered by 1 would be 0, which switches
        // dynamic D off, so the hint falls back to the single base D.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[("rollPID", "40,50,20"), ("d_min", "1,0,0")],
        ));
        assert!(pid.is_dmax_enabled());
        assert_eq!(
            pid.roll.near_optimal_d_hint(true).as_deref(),
            Some("D≈19 [optional D−1]")
        );
        // Base D of 1 lowered by 1 is 0: no hint at all.
        let one = AxisPid {
            d: Some(1),
            ..Default::default()
        };
        assert_eq!(one.near_optimal_d_hint(false), None);
    }

    #[test]
    fn test_near_optimal_hint_never_collapses_an_active_range() {
        // BF 4.5, D 2 is the upper end and d_min 1 the lower: lowering D to 1 would make the
        // ends equal (d_min < D fails), so there is no hint, and the console and legend
        // print the "none" line.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[("rollPID", "40,50,2"), ("d_min", "1,0,0")],
        ));
        assert!(pid.is_dmax_enabled());
        assert_eq!(pid.roll.near_optimal_d_hint(true), None);
        assert_eq!(
            pid.roll.near_optimal_recommendation_line(true),
            "Recommendation (none): No obvious tuning adjustments needed"
        );
        // BF 2025.12: D 2 is the lower end and d_max 3 the upper: both ends can drop by 1.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2025.12.0",
            &[("rollPID", "40,50,2"), ("d_max", "3,0,0")],
        ));
        assert_eq!(
            pid.roll.near_optimal_recommendation_line(true),
            "Recommendation (conservative): D-Min≈1, D-Max≈2 [optional D−1]"
        );
    }

    #[test]
    fn test_goal_d_never_returns_an_inverted_range_for_a_zero_base() {
        // P 40, D 20, range 1 to 20, target ratio 1000: the recommended base D rounds to 0.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[("rollPID", "40,50,20"), ("d_min", "1,0,0")],
        ));
        let (d, d_min, d_max) = pid.roll.calculate_goal_d_with_range(1000.0, true);
        assert_eq!((d, d_min, d_max), (Some(0), None, None));
    }

    #[test]
    fn test_recommended_d_min_never_rounds_to_zero() {
        // P 40, D 20, d_min 1: a target D of 4 scales D-Min to 0.2, which must stay at 1.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.5.0",
            &[("rollPID", "40,50,20"), ("d_min", "1,0,0")],
        ));
        let (d, d_min, d_max) = pid.roll.calculate_goal_d_with_range(10.0, true);
        assert_eq!((d, d_min, d_max), (Some(4), Some(1), Some(4)));
    }

    #[test]
    fn test_title_and_report_agree_per_axis_for_parsed_headers() {
        let sets: [&[(&str, &str)]; 4] = [
            // range on roll, d_min 0 on pitch, yaw D 0
            &[
                ("rollPID", "40,50,30"),
                ("pitchPID", "42,52,40"),
                ("yawPID", "45,80,0"),
                ("d_min", "20,0,0"),
            ],
            // 2025.12+: range on roll and pitch (equal), yaw D 0
            &[
                ("rollPID", "47,76,80"),
                ("pitchPID", "67,96,106"),
                ("yawPID", "35,65,0"),
                ("d_max", "82,106,0"),
            ],
            // slider at 0: d_max equals D on every axis
            &[
                ("rollPID", "45,80,30"),
                ("pitchPID", "47,84,34"),
                ("yawPID", "45,80,0"),
                ("d_max", "30,34,0"),
            ],
            // yaw-only range
            &[
                ("rollPID", "40,50,30"),
                ("pitchPID", "42,52,40"),
                ("yawPID", "45,80,25"),
                ("d_min", "0,0,15"),
            ],
        ];
        for extra in sets {
            let pid = parse_pid_metadata(&headers("Betaflight 4.5.0", extra));
            let on = pid.is_dmax_enabled();
            for axis in [&pid.roll, &pid.pitch, &pid.yaw] {
                let title = axis.format_for_title(&pid.firmware_type, on);
                let report = axis.format_for_report(&pid.firmware_type, on);
                match axis.d_range_for_display(on) {
                    Some((min, max)) => {
                        assert!(title.contains(&format!("D:{min}/{max}")), "{title}");
                        assert!(
                            report.contains(&format!("D-Min:{min} D-Max:{max}")),
                            "{report}"
                        );
                    }
                    None => {
                        assert!(
                            !title.contains('/') && !report.contains("D-Min"),
                            "{title} | {report}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_dynamic_d_not_active_without_range() {
        // BF 4.2.5 real log: d_min all zero means the firmware does not use dynamic D.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.2.5 (afdac08b3) STM32F411",
            &[
                ("rollPID", "69,94,50"),
                ("d_min", "0,0,0"),
                ("d_min_gain", "37"),
                ("d_min_advance", "20"),
            ],
        ));
        assert!(!pid.is_dmax_enabled());
        assert_eq!(
            pid.roll
                .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
            "P:69 I:94 D:50"
        );
        // 2025.12+: d_max equal to D means no boost range.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 2025.12.0",
            &[("rollPID", "38,81,35"), ("d_max", "35,0,0")],
        ));
        assert!(!pid.is_dmax_enabled());
        // No D-Min/D-Max header at all.
        let pid = parse_pid_metadata(&headers("Betaflight 4.4.0", &[("rollPID", "51,55,47")]));
        assert!(!pid.is_dmax_enabled());
        assert_eq!(
            pid.roll
                .format_for_report(&pid.firmware_type, pid.is_dmax_enabled()),
            "P:51 I:55 D:47"
        );
    }

    #[test]
    fn test_dynamic_d_does_not_depend_on_boost_settings() {
        // Zero gain and advance only remove the boost; the firmware still applies D-Min.
        let pid = parse_pid_metadata(&headers(
            "Betaflight 4.4.0",
            &[
                ("rollPID", "51,55,47"),
                ("d_min", "40,0,0"),
                ("d_max_gain", "0"),
                ("d_max_advance", "0"),
            ],
        ));
        assert!(pid.is_dmax_enabled());
    }

    // Spec for the same-D rule: a tier whose D target equals the current D is dropped.
    // Tier ratios are the current P:D times the tier multiplier (see constants.rs).
    use crate::constants::{
        PD_RATIO_AGGRESSIVE_MULTIPLIER, PD_RATIO_MODERATE_MULTIPLIER, PEAK_ACCEPTABLE_MULTIPLIER,
    };

    fn roll_axis(rev: &str, extra: &[(&str, &str)], roll_pid: &str) -> (PidMetadata, bool) {
        let mut h = vec![("rollPID", roll_pid)];
        h.extend_from_slice(extra);
        let pid = parse_pid_metadata(&headers(rev, &h));
        let on = pid.is_dmax_enabled();
        (pid, on)
    }

    fn tier_target(axis: &AxisPid, multiplier: f64, on: bool) -> DTarget {
        axis.calculate_goal_d_with_range(axis.calculate_pd_ratio().unwrap() * multiplier, on)
    }

    // P 40, D 12: the 2% step gives D = 12 / 0.98 = 12.24, which rounds to 12 (dropped).
    // The moderate step gives D = 12 / 0.75 = 16 (kept).
    #[test]
    fn same_d_conservative_tier_is_dropped_and_moderate_is_kept() {
        let (pid, on) = roll_axis("Betaflight 4.4.0", &[], "40,50,12");
        let cons = tier_target(&pid.roll, PEAK_ACCEPTABLE_MULTIPLIER, on);
        let moderate = tier_target(&pid.roll, PD_RATIO_MODERATE_MULTIPLIER, on);
        assert_eq!(cons.0, Some(12));
        assert_eq!(moderate.0, Some(16));
        let v = pid
            .roll
            .d_tier_visibility(Some(cons), Some(moderate), None, on);
        assert!(!v.conservative);
        assert!(v.moderate);
        assert!(!v.within_rounding_note);
    }

    // Only a conservative tier exists and it names D 12: one note replaces it.
    #[test]
    fn same_d_all_tiers_dropped_gives_the_note() {
        let (pid, on) = roll_axis("Betaflight 4.4.0", &[], "40,50,12");
        let cons = tier_target(&pid.roll, PEAK_ACCEPTABLE_MULTIPLIER, on);
        let v = pid.roll.d_tier_visibility(Some(cons), None, None, on);
        assert_eq!(
            v,
            DTierVisibility {
                conservative: false,
                moderate: false,
                aggressive: false,
                within_rounding_note: true
            }
        );
        // No conservative tier at all (no recommendation): no note.
        let v = pid.roll.d_tier_visibility(None, None, None, on);
        assert!(!v.within_rounding_note);
    }

    // Large D: the 2% step moves D by at least 0.5, so every tier differs from the current D.
    #[test]
    fn same_d_high_d_tiers_are_all_kept() {
        for d in 25u32..=300 {
            for ratio_x10 in [10u32, 14, 20, 35] {
                let p = d * ratio_x10 / 10;
                let (pid, on) = roll_axis("Betaflight 4.4.0", &[], &format!("{p},50,{d}"));
                let t = |m| tier_target(&pid.roll, m, on);
                let v = pid.roll.d_tier_visibility(
                    Some(t(PEAK_ACCEPTABLE_MULTIPLIER)),
                    Some(t(PD_RATIO_MODERATE_MULTIPLIER)),
                    Some(t(PD_RATIO_AGGRESSIVE_MULTIPLIER)),
                    on,
                );
                assert_eq!(
                    (
                        v.conservative,
                        v.moderate,
                        v.aggressive,
                        v.within_rounding_note
                    ),
                    (true, true, true, false),
                    "P {p} D {d}"
                );
            }
        }
    }

    // Range: D-Max 12 and D-Min 9. The 2% step keeps both ends (12, 9), so it is dropped.
    // The moderate step scales both ends by 16 / 12, so it differs.
    #[test]
    fn same_d_range_unchanged_is_dropped_changed_is_kept() {
        let (pid, on) = roll_axis("Betaflight 4.5.0", &[("d_min", "9,0,0")], "40,50,12");
        assert!(on);
        let cons = tier_target(&pid.roll, PEAK_ACCEPTABLE_MULTIPLIER, on);
        assert_eq!((cons.1, cons.2), (Some(9), Some(12)));
        let moderate = tier_target(&pid.roll, PD_RATIO_MODERATE_MULTIPLIER, on);
        assert_eq!((moderate.1, moderate.2), (Some(12), Some(16)));
        let v = pid
            .roll
            .d_tier_visibility(Some(cons), Some(moderate), None, on);
        assert!(!v.conservative && v.moderate);
    }

    // Range with one end changed: P 50, D-Max 25, D-Min 12. The 2% step gives D-Max
    // 25.51 -> 26 (changed) and D-Min 12.5 -> 12 (same); the tier stays because an end changed.
    #[test]
    fn same_d_range_with_one_changed_end_is_kept() {
        let (pid, on) = roll_axis("Betaflight 4.5.0", &[("d_min", "12,0,0")], "50,50,25");
        let cons = tier_target(&pid.roll, PEAK_ACCEPTABLE_MULTIPLIER, on);
        assert_eq!((cons.1, cons.2), (Some(12), Some(26)));
        let v = pid.roll.d_tier_visibility(Some(cons), None, None, on);
        assert!(v.conservative && !v.within_rounding_note);
    }

    // Unknown D target (no D known) is not "current": the tier stays as before.
    #[test]
    fn same_d_unknown_target_is_kept() {
        let (pid, on) = roll_axis("Betaflight 4.4.0", &[], "40,50,12");
        let v = pid
            .roll
            .d_tier_visibility(Some((None, None, None)), None, None, on);
        assert!(v.conservative);
    }
}
