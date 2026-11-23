// src/data_analysis/optimal_p_estimation.rs
//
// Optimal P Estimation Module
//
// Provides physics-aware P gain recommendations based on:
// - Frame-class-specific Td (time to 50%) targets
// - High-frequency noise level analysis
// - Response consistency metrics
//
// Implements theory from BrianWhite (PIDtoolbox) that optimal response timing
// is aircraft-specific, determined by power-to-rotational-inertia ratio.

use crate::constants::*;
use crate::data_input::pid_metadata::d_target_is_current;

/// Error type for optimal P analysis
#[derive(Debug, Clone)]
pub enum AnalysisError {
    /// No Td target available for the given frame class
    MissingTdTarget { message: String },
}

impl std::fmt::Display for AnalysisError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AnalysisError::MissingTdTarget { message } => {
                write!(f, "{}", message)
            }
        }
    }
}

impl std::error::Error for AnalysisError {}

/// Safe conversion from scaled f64 to u32 with saturation.
///
/// Computes (base * multiplier) and returns a saturated `u32` result:
/// - If `multiplier` is NaN, returns 0 (deterministic handling of invalid multiplier).
/// - If the scaled value is >= `u32::MAX`, returns `u32::MAX`.
/// - If the scaled value is <= 0.0, returns 0.
/// - Otherwise returns the truncated `u32` value.
fn safe_scaled_p(base: u32, multiplier: f64) -> u32 {
    // Explicitly handle NaN deterministically rather than relying on cast behavior
    if multiplier.is_nan() {
        return 0;
    }

    let scaled = (base as f64) * multiplier;
    if scaled.is_infinite() {
        if scaled.is_sign_positive() {
            return u32::MAX;
        } else {
            return 0;
        }
    }

    if scaled >= (u32::MAX as f64) {
        u32::MAX
    } else if scaled <= 0.0 {
        0
    } else {
        scaled as u32
    }
}

/// Scales `base` up by `multiplier` and guarantees a real increase.
///
/// `safe_scaled_p` truncates, so a small `base` (for example P 10 with +5%) scales to the
/// same integer. An increase recommendation must name a higher P, so the result is at least
/// `base + 1` (saturating at `u32::MAX`). For larger `base` the scaled value is already higher
/// and is returned unchanged.
fn safe_raised_p(base: u32, multiplier: f64) -> u32 {
    safe_scaled_p(base, multiplier).max(base.saturating_add(1))
}

/// Noise level classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NoiseLevel {
    Low,      // < 10% HF energy
    Moderate, // 10-15% HF energy
    High,     // > 15% HF energy
    Unknown,  // Cannot determine
}

impl NoiseLevel {
    pub fn name(&self) -> &str {
        match self {
            NoiseLevel::Low => "LOW",
            NoiseLevel::Moderate => "MODERATE",
            NoiseLevel::High => "HIGH",
            NoiseLevel::Unknown => "UNKNOWN",
        }
    }

    #[allow(dead_code)]
    pub fn assessment(&self) -> &str {
        match self {
            NoiseLevel::Low => "Noise levels are acceptable, P has headroom",
            NoiseLevel::Moderate => "Approaching noise limits",
            NoiseLevel::High => "At or exceeding recommended noise limits",
            NoiseLevel::Unknown => "Cannot assess noise levels (D-term data unavailable)",
        }
    }
}

/// Td deviation classification
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TdDeviation {
    SignificantlySlower, // > 30% slower than target
    ModeratelySlower,    // 15-30% slower
    WithinTarget,        // ±15% of target
    SignificantlyFaster, // < -15% faster than target
}

impl TdDeviation {
    pub fn name(&self) -> &str {
        match self {
            TdDeviation::SignificantlySlower => "SIGNIFICANTLY SLOWER",
            TdDeviation::ModeratelySlower => "MODERATELY SLOWER",
            TdDeviation::WithinTarget => "WITHIN TARGET",
            TdDeviation::SignificantlyFaster => "FASTER",
        }
    }
}

/// P optimization recommendation
#[derive(Debug, Clone, PartialEq)]
pub enum PRecommendation {
    Optimal {
        reasoning: String,
        /// True when Td is already faster than the physics target: more P headroom is likely
        /// but cannot be quantified, so the short text says so instead of "optimal".
        headroom_unquantified: bool,
    },
    Increase {
        conservative_p: u32,
        reasoning: String,
    },
    Decrease {
        recommended_p: u32,
        reasoning: String,
    },
    Investigate {
        issue: String,
    },
}

impl PRecommendation {
    /// Short text for an `Optimal` recommendation, shared by the console, the legend and the
    /// report, so all three say the same thing. `None` for the other variants.
    pub fn optimal_short_text(&self) -> Option<String> {
        match self {
            PRecommendation::Optimal {
                headroom_unquantified: true,
                ..
            } => Some("No change suggested, headroom possible".to_string()),
            PRecommendation::Optimal { .. } => Some("Current P is optimal".to_string()),
            _ => None,
        }
    }
}

/// Statistics for Td measurements across multiple step responses
#[derive(Debug, Clone)]
pub struct TdStatistics {
    /// Median Td in ms. The median is used because single windows can fail deconvolution and
    /// return Td values many times the typical one; a mean would follow those outliers.
    pub median_ms: f64,
    /// Robust coefficient of variation: `TD_MAD_TO_SIGMA * MAD / median`. `None` when there are
    /// too few samples.
    pub coefficient_of_variation: Option<f64>,
    pub num_samples: usize,
    /// Fraction of samples within `TD_COEFFICIENT_OF_VARIATION_MAX * median` of the median.
    pub consistency: f64,
}

/// Median of a non-empty slice already sorted ascending.
fn sorted_median(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n.is_multiple_of(PARITY_DIVISOR) {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    } else {
        sorted[n / 2]
    }
}

impl TdStatistics {
    /// Calculate statistics from array of Td values (in milliseconds)
    pub fn from_samples(td_samples_ms: &[f64]) -> Option<Self> {
        let mut sorted: Vec<f64> = td_samples_ms
            .iter()
            .copied()
            .filter(|x| x.is_finite())
            .collect();

        if sorted.is_empty() {
            return None;
        }
        sorted.sort_by(|a, b| a.total_cmp(b));

        let n = sorted.len() as f64;
        let median = sorted_median(&sorted);

        // Use epsilon-based comparison to avoid division by near-zero values
        if median.abs() <= TD_MEAN_EPSILON {
            return None;
        }

        // Robust spread: median absolute deviation, scaled to match a standard deviation for
        // normally distributed data. A few outlier windows do not inflate it.
        // Too few samples: no spread estimate (coefficient_of_variation None).
        let coefficient_of_variation = if sorted.len() < TD_SAMPLES_MIN_FOR_STDDEV {
            None
        } else {
            let mut abs_dev: Vec<f64> = sorted.iter().map(|&x| (x - median).abs()).collect();
            abs_dev.sort_by(|a, b| a.total_cmp(b));
            Some(TD_MAD_TO_SIGMA * sorted_median(&abs_dev) / median)
        };

        // Consistency: fraction of samples within the allowed relative spread of the median.
        // When coefficient_of_variation is None (too few samples), consistency is perfect (1.0).
        let consistency = coefficient_of_variation.map_or(1.0, |_| {
            let tolerance = TD_COEFFICIENT_OF_VARIATION_MAX * median.abs();
            let within_range = sorted
                .iter()
                .filter(|&&x| (x - median).abs() <= tolerance)
                .count();
            within_range as f64 / n
        });

        Some(TdStatistics {
            median_ms: median,
            coefficient_of_variation,
            num_samples: sorted.len(),
            consistency,
        })
    }

    /// Consistency and CV text with their thresholds, shared by the plot legend, console and
    /// report, for example `Consistency=86% (⊢≥70%), CV=24.8% (⊢≤40%)`.
    pub fn reliability_details(&self) -> String {
        let cv_str = self.coefficient_of_variation.map_or_else(
            || "CV=N/A".to_string(),
            |cv| {
                format!(
                    "CV={:.1}% (⊢≤{:.0}%)",
                    cv * 100.0,
                    TD_COEFFICIENT_OF_VARIATION_MAX * 100.0,
                )
            },
        );
        format!(
            "Consistency={:.0}% (⊢≥{:.0}%), {cv_str}",
            self.consistency * 100.0,
            TD_CONSISTENCY_MIN_THRESHOLD * 100.0,
        )
    }

    /// `Reliable` or `Unreliable`, from `is_consistent`.
    pub fn reliability_label(&self) -> &'static str {
        if self.is_consistent() {
            "Reliable"
        } else {
            "Unreliable"
        }
    }

    /// Check if measurements are consistent enough for reliable analysis
    pub fn is_consistent(&self) -> bool {
        // Need at least 2 samples for meaningful consistency check
        self.num_samples >= TD_SAMPLES_MIN_FOR_STDDEV
            && self.consistency >= TD_CONSISTENCY_MIN_THRESHOLD
            && self
                .coefficient_of_variation
                .is_none_or(|cv| cv <= TD_COEFFICIENT_OF_VARIATION_MAX)
    }
}

/// Complete optimal P analysis result
#[derive(Debug, Clone)]
pub struct OptimalPAnalysis {
    pub current_p: u32,
    pub current_d: Option<u32>,
    pub recommended_pd_conservative: Option<f64>,
    pub td_stats: TdStatistics,
    pub td_deviation: TdDeviation,
    pub td_deviation_percent: f64,
    pub noise_level: NoiseLevel,
    pub recommendation: PRecommendation,
    /// Actual Td target (in ms) used during analysis (from physics)
    pub td_target_ms: f64,
    /// Actual Td tolerance (in ms) used during analysis (from physics)
    pub td_tolerance_ms: f64,
    /// Number of throttle-punch events used to derive the physics Td target.
    pub source_events: usize,
    /// Number of flight files that contributed to the physics Td target.
    pub source_files: usize,
    /// Spread of the throttle-punch torque/inertia ratios (half inter-quartile range / median).
    /// A wide spread means the punches disagree, so the Td target is less certain.
    pub source_spread: Option<f64>,
    /// Seconds of flight data the step-response windows came from (log time after the --start/--end
    /// trim and the fixed start/end margins). Shown on the reliability line.
    pub analysed_seconds: Option<f64>,
}

impl OptimalPAnalysis {
    /// Analyze optimal P for a given axis
    ///
    /// # Arguments
    /// * `td_samples_ms` - Array of Td measurements from multiple step responses (milliseconds)
    /// * `current_p` - Current P gain
    /// * `current_d` - Current D gain (optional)
    /// * `hf_energy_ratio` - Optional: ratio of D-term energy above DTERM_HF_CUTOFF_HZ (0.0-1.0)
    /// * `recommended_pd_conservative` - Optional: recommended P:D ratio from step response (conservative)
    /// * `physics_td_target_ms` - Physics-derived (td_target, tolerance) from torque_inertia_profiler
    pub fn analyze(
        td_samples_ms: &[f64],
        current_p: u32,
        current_d: Option<u32>,
        hf_energy_ratio: Option<f64>,
        recommended_pd_conservative: Option<f64>,
        physics_td_target_ms: Option<(f64, f64)>,
    ) -> Result<Self, AnalysisError> {
        // Calculate Td statistics
        let td_stats = TdStatistics::from_samples(td_samples_ms).ok_or_else(|| {
            AnalysisError::MissingTdTarget {
                message: "Failed to calculate Td statistics from samples.".to_string(),
            }
        })?;

        // Get target Td — use physics-based value or error
        let (td_target_ms, td_tolerance_ms) =
            if let Some((phys_target, phys_tol)) = physics_td_target_ms {
                (phys_target, phys_tol)
            } else {
                return Err(AnalysisError::MissingTdTarget {
                    message: "No physics-derived Td target available. \
                              Ensure throttle-punch events were detected (need \
                              \u{2265}5 events)."
                        .to_string(),
                });
            };

        // Defensive check: td_target_ms must be above domain minimum to be physically meaningful
        if td_target_ms <= MIN_TD_MS {
            return Err(AnalysisError::MissingTdTarget {
                message: format!(
                    "Invalid Td target ({:.3}ms, minimum {:.3}ms) for optimal P analysis. Skipping.",
                    td_target_ms, MIN_TD_MS
                ),
            });
        }

        // Calculate deviation from target (safe: td_target_ms validated above)
        let td_deviation_percent = ((td_stats.median_ms - td_target_ms) / td_target_ms) * 100.0;

        // Classify deviation
        let td_deviation = if td_deviation_percent > TD_DEVIATION_SIGNIFICANTLY_SLOWER_THRESHOLD {
            TdDeviation::SignificantlySlower
        } else if td_deviation_percent > TD_DEVIATION_MODERATELY_SLOWER_THRESHOLD {
            TdDeviation::ModeratelySlower
        } else if td_deviation_percent < TD_DEVIATION_SIGNIFICANTLY_FASTER_THRESHOLD {
            TdDeviation::SignificantlyFaster
        } else {
            TdDeviation::WithinTarget
        };

        // Classify noise level
        let noise_level = if let Some(hf_ratio) = hf_energy_ratio {
            if hf_ratio < DTERM_HF_ENERGY_MODERATE {
                NoiseLevel::Low
            } else if hf_ratio < DTERM_HF_ENERGY_THRESHOLD {
                NoiseLevel::Moderate
            } else {
                NoiseLevel::High
            }
        } else {
            NoiseLevel::Unknown
        };

        // Generate recommendation based on Td deviation and noise level
        let recommendation = Self::generate_recommendation(
            current_p,
            &td_deviation,
            td_deviation_percent,
            &noise_level,
            &td_stats,
        );

        Ok(OptimalPAnalysis {
            current_p,
            current_d,
            recommended_pd_conservative,
            td_stats,
            td_deviation,
            td_deviation_percent,
            noise_level,
            recommendation,
            td_target_ms,
            td_tolerance_ms,
            source_events: 0,
            source_files: 0,
            source_spread: None,
            analysed_seconds: None,
        })
    }

    /// Generate P recommendation based on analysis
    fn generate_recommendation(
        current_p: u32,
        td_deviation: &TdDeviation,
        td_deviation_percent: f64,
        noise_level: &NoiseLevel,
        td_stats: &TdStatistics,
    ) -> PRecommendation {
        match (td_deviation, noise_level) {
            // Case 1: Td significantly slower + low noise = clear headroom to increase P
            (TdDeviation::SignificantlySlower, NoiseLevel::Low) => {
                let conservative = safe_raised_p(current_p, P_HEADROOM_MODERATE_MULTIPLIER);
                PRecommendation::Increase {
                    conservative_p: conservative,
                    reasoning: format!(
                        "Response is {:.1}% slower than target with low noise levels. \
                         P can be increased significantly for faster response.",
                        td_deviation_percent
                    ),
                }
            }

            // Case 2: Td moderately slower + low/moderate noise = modest headroom
            (TdDeviation::ModeratelySlower, NoiseLevel::Low | NoiseLevel::Moderate) => {
                let conservative = safe_raised_p(current_p, P_HEADROOM_CONSERVATIVE_MULTIPLIER);
                PRecommendation::Increase {
                    conservative_p: conservative,
                    reasoning: format!(
                        "Response is {:.1}% slower than target. Modest P increase recommended.",
                        td_deviation_percent
                    ),
                }
            }

            // Case 2b: Td moderately slower + high noise = investigate
            (TdDeviation::ModeratelySlower, NoiseLevel::High) => PRecommendation::Investigate {
                issue: format!(
                    "Response is {:.1}% slower than target despite high noise levels. \
                     This suggests mechanical issues (damaged props, loose hardware, motor problems) \
                     or incorrect frame class. Inspect aircraft.",
                    td_deviation_percent
                ),
            },

            // Case 3: Td within target + low noise = slight headroom, but only when Td is not
            // already faster than the target (deviation >= 0). A faster-than-target response
            // has no headroom left by the physics target.
            (TdDeviation::WithinTarget, NoiseLevel::Low) => {
                // Plain scaling: the `conservative > current_p` guard below must stay able to
                // reject a step that truncates to the same P (no headroom to claim).
                let conservative = safe_scaled_p(current_p, P_HEADROOM_CONSERVATIVE_MULTIPLIER);
                if td_deviation_percent >= 0.0 && conservative > current_p {
                    PRecommendation::Increase {
                        conservative_p: conservative,
                        reasoning: "Response time is in target range with low noise. \
                             Minor P increase possible if seeking faster response.".to_string(),
                    }
                } else {
                    PRecommendation::Optimal {
                headroom_unquantified: false,
                        reasoning: format!(
                            "Response time is in target range ({:.1}ms) with low noise levels. \
                             Current P ({}) appears optimal.",
                            td_stats.median_ms, current_p
                        ),
                    }
                }
            }

            // Case 4: Td within target + moderate noise = likely optimal
            (TdDeviation::WithinTarget, NoiseLevel::Moderate) => PRecommendation::Optimal {
                headroom_unquantified: false,
                reasoning: format!(
                    "Response time is in target range ({:.1}ms) with moderate noise levels. \
                     Current P ({}) appears optimal for this aircraft.",
                    td_stats.median_ms, current_p
                ),
            },

            // Case 5: Td within target + high noise = optimal but monitor
            (TdDeviation::WithinTarget, NoiseLevel::High) => PRecommendation::Optimal {
                headroom_unquantified: false,
                reasoning: format!(
                    "Response time is in target range but noise levels are high. \
                     Current P ({}) is at or near physical limits. Monitor motor temperatures.",
                    current_p
                ),
            },

            // Case 6: Td faster than target + high noise = at limit, consider reduction
            (TdDeviation::SignificantlyFaster, NoiseLevel::High) => {
                let recommended = safe_scaled_p(current_p, P_REDUCTION_MODERATE_MULTIPLIER);
                PRecommendation::Decrease {
                    recommended_p: recommended,
                    reasoning: format!(
                        "Response is {:.1}% faster than target with high noise levels. \
                         P may be exceeding physical limits. Consider reduction if motors overheat.",
                        td_deviation_percent
                    ),
                }
            }

            // Case 7: Td faster than target + moderate noise = at optimal limit
            (TdDeviation::SignificantlyFaster, NoiseLevel::Moderate) => PRecommendation::Optimal {
                headroom_unquantified: false,
                reasoning: format!(
                    "Response is {:.1}% faster than target with moderate noise. \
                     Current P ({}) is at or near optimal limit for this aircraft.",
                    td_deviation_percent, current_p
                ),
            },

            // Case 8: Td faster than target + low noise = no P change suggested. The target is
            // conservative for this aircraft, so more headroom is likely but cannot be
            // quantified (the target depends on the punch pool and an empirical factor).
            (TdDeviation::SignificantlyFaster, NoiseLevel::Low) => PRecommendation::Optimal {
                headroom_unquantified: true,
                reasoning: format!(
                    "Response is {:.1}% faster than target with low noise. \
                     The target may be conservative for this aircraft, so more P headroom is \
                     possible but not quantified. Test any increase in small steps and check \
                     motor temperature and noise. The prop or build may differ from the spec.",
                    td_deviation_percent
                ),
            },

            // Case 9: Td significantly slower + moderate/high noise = investigate
            (TdDeviation::SignificantlySlower, NoiseLevel::Moderate | NoiseLevel::High) => {
                PRecommendation::Investigate {
                    issue: format!(
                        "Response is {:.1}% slower than target despite moderate/high noise. \
                         This suggests mechanical issues (damaged props, loose hardware, \
                         motor problems) or incorrect frame class. Inspect aircraft.",
                        td_deviation_percent
                    ),
                }
            }

            // Case 10: Unknown noise level - provide Td-only recommendation
            (_, NoiseLevel::Unknown) => match td_deviation {
                TdDeviation::SignificantlySlower | TdDeviation::ModeratelySlower => {
                    let conservative =
                        safe_raised_p(current_p, P_HEADROOM_CONSERVATIVE_MULTIPLIER);
                    PRecommendation::Increase {
                        conservative_p: conservative,
                        reasoning: format!(
                            "Response is {:.1}% slower than target. P increase recommended, \
                             but monitor motor temperatures (D-term data unavailable for noise analysis).",
                            td_deviation_percent
                        ),
                    }
                }
                TdDeviation::WithinTarget => PRecommendation::Optimal {
                headroom_unquantified: false,
                    reasoning: format!(
                        "Response time ({:.1}ms) is in target range. Current P ({}) appears appropriate, \
                         but monitor motor temperatures (D-term data unavailable for noise analysis).",
                        td_stats.median_ms, current_p
                    ),
                },
                TdDeviation::SignificantlyFaster => PRecommendation::Optimal {
                headroom_unquantified: false,
                    reasoning: format!(
                        "Response is {:.1}% faster than target. Current P ({}) may be at limits, \
                         monitor motor temperatures (D-term data unavailable for noise analysis).",
                        td_deviation_percent, current_p
                    ),
                },
            },
        }
    }

    /// Flight clip text shared by the legend, console and report, for example `218s`: the seconds
    /// of log time the step-response windows came from. `None` when unknown.
    pub fn flight_clip_text(&self) -> Option<String> {
        self.analysed_seconds.map(|s| format!("{s:.0}s"))
    }

    /// Td source text shared by the legend, console and report, for example
    /// `4 flights, 14 punches, spread 32%`. The spread is the half inter-quartile range of the
    /// punch ratios over their median; it is omitted when there are too few punches to compute it.
    pub fn source_summary(&self) -> String {
        let flights = if self.source_files == 1 {
            "flight"
        } else {
            "flights"
        };
        let spread = self
            .source_spread
            .map_or_else(String::new, |f| format!(", spread {:.0}%", f * 100.0));
        format!(
            "{} {}, {} punches{}",
            self.source_files, flights, self.source_events, spread
        )
    }

    /// Recommended-D text appended to a P recommendation, shared by the console and the plot
    /// legend. Scales the D that matches the P:D ratio: base D as `, D≈x (+n)`, or, when
    /// `d_range` holds the axis D-Min and D-Max (dynamic D enabled), both ends by the same
    /// factor as `, D-Min≈a (+n), D-Max≈b (+n)`. Empty when D or the ratio is unknown, or when
    /// the target equals the current D (`d_target_is_current`): no D change to name.
    pub fn d_target_suffix(&self, recommended_p: u32, d_range: Option<(u32, u32)>) -> String {
        let effective_pd = self.recommended_pd_conservative.or_else(|| {
            self.current_d
                .filter(|&d| d > 0)
                .map(|d| self.current_p as f64 / d as f64)
        });
        let (Some(current_d), Some(rec_pd)) = (self.current_d, effective_pd) else {
            return String::new();
        };
        if rec_pd <= 0.0 || current_d == 0 {
            return String::new();
        }
        let rec_d = (recommended_p as f64 / rec_pd).round() as u32;
        // AxisPid::calculate_goal_d_with_range scales from the axis' current P; here the
        // target D follows the recommended P, so the ends are scaled directly.
        // A target D that rounds to 0 has no representable range: show the single D.
        if let Some((min, max)) = d_range.filter(|_| rec_d > 0) {
            let scale = rec_d as f64 / current_d as f64;
            // A recommended D-Min of 0 would switch dynamic D off.
            let rec_min = ((min as f64 * scale).round() as u32).max(1);
            let rec_max = (max as f64 * scale).round() as u32;
            if d_target_is_current(
                (Some(current_d), Some(min), Some(max)),
                (Some(rec_d), Some(rec_min), Some(rec_max)),
                true,
            ) {
                return String::new();
            }
            return format!(
                ", D-Min≈{} ({:+}), D-Max≈{} ({:+})",
                rec_min,
                rec_min as i64 - min as i64,
                rec_max,
                rec_max as i64 - max as i64
            );
        }
        if d_target_is_current(
            (Some(current_d), None, None),
            (Some(rec_d), None, None),
            false,
        ) {
            return String::new();
        }
        format!(", D≈{} ({:+})", rec_d, rec_d as i64 - current_d as i64)
    }

    /// Format analysis as human-readable console output.
    ///
    /// Order: recommendation first, then the evidence behind it (same order as the legend and
    /// the report).
    pub fn format_console_output(&self, axis_name: &str, d_range: Option<(u32, u32)>) -> String {
        let mut output = String::new();
        output.push_str(&format!("{axis_name}:\n"));

        match &self.recommendation {
            rec @ PRecommendation::Optimal { .. } => {
                output.push_str(&format!(
                    "  Recommendation: {}\n",
                    rec.optimal_short_text().unwrap_or_default()
                ));
            }
            PRecommendation::Increase { conservative_p, .. } => {
                let p_delta = *conservative_p as i32 - self.current_p as i32;
                output.push_str(&format!(
                    "  Recommendation (Conservative): P≈{} ({:+}){}\n",
                    conservative_p,
                    p_delta,
                    self.d_target_suffix(*conservative_p, d_range),
                ));
            }
            PRecommendation::Decrease { recommended_p, .. } => {
                let p_delta = *recommended_p as i32 - self.current_p as i32;
                output.push_str(&format!(
                    "  Recommendation (Decrease): P≈{} ({:+}){}\n",
                    recommended_p,
                    p_delta,
                    self.d_target_suffix(*recommended_p, d_range),
                ));
            }
            PRecommendation::Investigate { issue } => {
                output.push_str(&format!("  Recommendation: Investigate — {}\n", issue));
            }
        }

        // The console has no axis title, so it keeps the current P (the legend drops it).
        output.push_str(&format!("  Current P={}\n", self.current_p));
        output.push_str(&format!(
            "  Td (median): {:.1}ms (target: {:.1}±{:.1}ms)\n",
            self.td_stats.median_ms, self.td_target_ms, self.td_tolerance_ms,
        ));
        if let Some(clip) = self.flight_clip_text() {
            output.push_str(&format!("  Flight clip: {clip}\n"));
        }

        let deviation_sign = if self.td_deviation_percent > 0.0 {
            "+"
        } else {
            ""
        };
        output.push_str(&format!(
            "  Deviation: {}{:.1}% ({})\n",
            deviation_sign,
            self.td_deviation_percent,
            self.td_deviation.name(),
        ));
        output.push_str(&format!("  Noise: {}\n", self.noise_level.name()));
        output.push_str(&format!("  Td source: {}\n", self.source_summary()));
        output.push_str(&format!(
            "  {}: {}\n",
            self.td_stats.reliability_label(),
            self.td_stats.reliability_details()
        ));

        output
    }
}

#[cfg(test)]
mod d_target_tests {
    use super::*;

    fn analysis(current_d: Option<u32>, rec_pd: Option<f64>) -> OptimalPAnalysis {
        let samples: Vec<f64> = (0..40).map(|i| 28.0 + (i % 5) as f64).collect();
        OptimalPAnalysis::analyze(&samples, 40, current_d, None, rec_pd, Some((25.0, 3.0)))
            .expect("analysis from synthetic Td samples")
    }

    #[test]
    fn d_target_suffix_scales_base_d() {
        // P 40, D 20, target P:D 2.0 -> recommended P 44 needs D = 44 / 2.0 = 22.
        let a = analysis(Some(20), Some(2.0));
        assert_eq!(a.d_target_suffix(44, None), ", D≈22 (+2)");
    }

    #[test]
    fn d_target_suffix_scales_both_ends_with_dynamic_d() {
        // Same D scale factor 22 / 20 = 1.1 applied to D-Min 20 and D-Max 30.
        let a = analysis(Some(20), Some(2.0));
        assert_eq!(
            a.d_target_suffix(44, Some((20, 30))),
            ", D-Min≈22 (+2), D-Max≈33 (+3)"
        );
        // Equal ends stay equal.
        assert_eq!(
            a.d_target_suffix(44, Some((20, 20))),
            ", D-Min≈22 (+2), D-Max≈22 (+2)"
        );
    }

    #[test]
    fn reliability_text_uses_the_shared_thresholds() {
        let stats = |samples: Vec<f64>| {
            OptimalPAnalysis::analyze(&samples, 40, Some(20), None, Some(2.0), Some((25.0, 3.0)))
                .expect("analysis from synthetic Td samples")
                .td_stats
        };
        // Identical samples: no spread, so the CV limit and consistency minimum are met.
        let steady = stats(vec![30.0; 40]);
        assert_eq!(steady.reliability_label(), "Reliable");
        let text = steady.reliability_details();
        assert!(text.starts_with("Consistency="), "{text}");
        assert!(
            text.contains("(⊢≥70%)") && text.contains("(⊢≤40%)"),
            "{text}"
        );
        // Samples alternating between 10 and 50 ms give a CV far above the 40% limit.
        let spread = stats(
            (0..40)
                .map(|i| if i % 2 == 0 { 10.0 } else { 50.0 })
                .collect(),
        );
        assert_eq!(spread.reliability_label(), "Unreliable");
    }

    fn low_noise_recommendation(td_ms: f64) -> PRecommendation {
        // Target 25 +- 3 ms, P 60, D 20, low D-term noise (1% high-frequency energy).
        OptimalPAnalysis::analyze(
            &[td_ms; 40],
            60,
            Some(20),
            Some(0.01),
            Some(2.0),
            Some((25.0, 3.0)),
        )
        .expect("analysis")
        .recommendation
    }

    #[test]
    fn within_target_low_noise_slower_than_target_gets_headroom() {
        // 26 ms is +4% against the 25 ms target.
        assert!(matches!(
            low_noise_recommendation(26.0),
            PRecommendation::Increase { .. }
        ));
        // Exactly on target (0%) still gets headroom.
        assert!(matches!(
            low_noise_recommendation(25.0),
            PRecommendation::Increase { .. }
        ));
    }

    #[test]
    fn optimal_short_text_flags_unquantified_headroom() {
        // 18 ms is -28% against the 25 ms target: Case 8.
        let faster = low_noise_recommendation(18.0);
        assert_eq!(
            faster.optimal_short_text().as_deref(),
            Some("No change suggested, headroom possible")
        );
        // Within the band and slightly faster than target: plain optimal wording.
        let within = low_noise_recommendation(23.5);
        assert_eq!(
            within.optimal_short_text().as_deref(),
            Some("Current P is optimal")
        );
        // Increase has no Optimal short text.
        assert_eq!(low_noise_recommendation(26.0).optimal_short_text(), None);
    }

    // Spec: the recommendation follows Td zone x noise level. Synthetic inputs only: target 25 ms
    // (+-3 ms), P 60, so +5% P is 63, +10% is 66 and -5% is 57. Zones: faster than -15% is
    // SignificantlyFaster, +-15% is WithinTarget, +15% to +30% is ModeratelySlower, beyond +30% is
    // SignificantlySlower.
    #[test]
    fn recommendation_table_follows_td_zone_and_noise() {
        #[derive(Debug, PartialEq)]
        enum Kind {
            Optimal,
            OptimalHeadroom,
            Increase(u32),
            Decrease(u32),
            Investigate,
        }
        let low = Some(DTERM_HF_ENERGY_MODERATE * 0.5);
        let moderate = Some((DTERM_HF_ENERGY_MODERATE + DTERM_HF_ENERGY_THRESHOLD) / 2.0);
        let high = Some(DTERM_HF_ENERGY_THRESHOLD * 1.5);
        let kind = |td_ms: f64, hf: Option<f64>| {
            let a = OptimalPAnalysis::analyze(
                &[td_ms; 40],
                60,
                Some(20),
                hf,
                Some(2.0),
                Some((25.0, 3.0)),
            )
            .expect("analysis");
            match a.recommendation {
                PRecommendation::Optimal {
                    headroom_unquantified: true,
                    ..
                } => Kind::OptimalHeadroom,
                PRecommendation::Optimal { .. } => Kind::Optimal,
                PRecommendation::Increase { conservative_p, .. } => Kind::Increase(conservative_p),
                PRecommendation::Decrease { recommended_p, .. } => Kind::Decrease(recommended_p),
                PRecommendation::Investigate { .. } => Kind::Investigate,
            }
        };
        // 18 ms is -28%, 21.5 is -14%, 25 is 0%, 28.7 is +14.8%, 30 is +20%, 33 is +32%.
        let table: [(f64, Option<f64>, Kind); 22] = [
            (18.0, low, Kind::OptimalHeadroom),
            (21.5, low, Kind::Optimal),
            (25.0, low, Kind::Increase(63)),
            (28.7, low, Kind::Increase(63)),
            (30.0, low, Kind::Increase(63)),
            (33.0, low, Kind::Increase(66)),
            (18.0, moderate, Kind::Optimal),
            (21.5, moderate, Kind::Optimal),
            (25.0, moderate, Kind::Optimal),
            (30.0, moderate, Kind::Increase(63)),
            (33.0, moderate, Kind::Investigate),
            (18.0, high, Kind::Decrease(57)),
            (21.5, high, Kind::Optimal),
            (25.0, high, Kind::Optimal),
            (28.7, high, Kind::Optimal),
            (30.0, high, Kind::Investigate),
            (33.0, high, Kind::Investigate),
            (18.0, None, Kind::Optimal),
            (25.0, None, Kind::Optimal),
            (28.7, None, Kind::Optimal),
            (30.0, None, Kind::Increase(63)),
            (33.0, None, Kind::Increase(63)),
        ];
        for (td, hf, expected) in table {
            assert_eq!(kind(td, hf), expected, "Td {td} ms, noise {hf:?}");
        }
    }

    #[test]
    fn significantly_faster_than_target_low_noise_is_optimal() {
        // 18 ms is -28% against the 25 ms target (below the -15% line).
        assert!(matches!(
            low_noise_recommendation(18.0),
            PRecommendation::Optimal { .. }
        ));
    }

    #[test]
    fn within_target_low_noise_faster_than_target_is_optimal() {
        // 23.5 ms is -6% against the 25 ms target: already faster, no headroom.
        assert!(matches!(
            low_noise_recommendation(23.5),
            PRecommendation::Optimal { .. }
        ));
    }

    #[test]
    fn flight_clip_text_shows_whole_seconds() {
        let samples = vec![30.0; 40];
        let mut a =
            OptimalPAnalysis::analyze(&samples, 40, Some(20), None, Some(2.0), Some((25.0, 3.0)))
                .expect("analysis");
        assert_eq!(a.flight_clip_text(), None);
        a.analysed_seconds = Some(217.6);
        assert_eq!(a.flight_clip_text().as_deref(), Some("218s"));
    }

    #[test]
    fn td_statistics_median_ignores_outlier_windows() {
        // 100 windows at 20 ms plus 3 failed windows at 200 ms.
        let mut samples = vec![20.0; 100];
        samples.extend([200.0; 3]);
        let stats = TdStatistics::from_samples(&samples).expect("stats");
        assert_eq!(stats.median_ms, 20.0);
        assert_eq!(stats.coefficient_of_variation, Some(0.0));
        assert!(
            stats.is_consistent(),
            "outliers must not make it Unreliable"
        );
    }

    #[test]
    fn td_statistics_median_of_even_count_is_midpoint() {
        let stats = TdStatistics::from_samples(&[40.0, 10.0, 30.0, 20.0]).expect("stats");
        assert_eq!(stats.median_ms, 25.0);
        // MAD of [15, 5, 5, 15] about 25 is 10; CV = 1.4826 * 10 / 25.
        let cv = stats.coefficient_of_variation.expect("cv");
        assert!((cv - TD_MAD_TO_SIGMA * 10.0 / 25.0).abs() < 1e-12, "{cv}");
    }

    #[test]
    fn td_statistics_wide_spread_stays_unreliable() {
        // Half the windows near 10 ms and half near 50 ms is a real spread, not outliers.
        let samples: Vec<f64> = (0..40)
            .map(|i| if i % 2 == 0 { 10.0 } else { 50.0 })
            .collect();
        let stats = TdStatistics::from_samples(&samples).expect("stats");
        assert!(!stats.is_consistent());
        assert_eq!(stats.consistency, 0.0);
    }

    #[test]
    fn td_statistics_rejects_empty_non_finite_and_zero_median_input() {
        assert!(TdStatistics::from_samples(&[]).is_none());
        assert!(TdStatistics::from_samples(&[f64::NAN, f64::INFINITY]).is_none());
        // Non-finite values are dropped; the rest are used.
        let stats = TdStatistics::from_samples(&[f64::NAN, 20.0, 30.0]).expect("stats");
        assert_eq!(stats.num_samples, 2);
        assert_eq!(stats.median_ms, 25.0);
        // A zero median has no relative spread.
        assert!(TdStatistics::from_samples(&[0.0, 0.0, 0.0]).is_none());
    }

    #[test]
    fn td_statistics_single_sample_has_no_spread() {
        let stats = TdStatistics::from_samples(&[22.0]).expect("stats");
        assert_eq!(stats.median_ms, 22.0);
        assert_eq!(stats.coefficient_of_variation, None);
        assert!(
            !stats.is_consistent(),
            "one sample is below the minimum count"
        );
    }

    #[test]
    fn d_target_suffix_shows_a_single_d_when_the_target_rounds_to_zero() {
        // P 40, D 20, P:D 2.0; recommended P 0 needs D 0, so a range cannot be shown.
        let a = analysis(Some(20), Some(2.0));
        assert_eq!(a.d_target_suffix(0, Some((1, 20))), ", D≈0 (-20)");
    }

    #[test]
    fn d_target_suffix_keeps_d_min_above_zero() {
        // P 40, D 20, P:D 2.0; recommended P 4 needs D 2, so D-Min 1 would scale to 0.1.
        let a = analysis(Some(20), Some(2.0));
        assert_eq!(
            a.d_target_suffix(4, Some((1, 20))),
            ", D-Min≈1 (+0), D-Max≈2 (-18)"
        );
    }

    #[test]
    fn source_summary_names_flights_punches_and_spread() {
        let mut a = analysis(Some(20), Some(2.0));
        a.source_files = 1;
        a.source_events = 5;
        assert_eq!(a.source_summary(), "1 flight, 5 punches");
        a.source_files = 11;
        a.source_events = 94;
        assert_eq!(a.source_summary(), "11 flights, 94 punches");
        a.source_spread = Some(0.214);
        assert_eq!(a.source_summary(), "11 flights, 94 punches, spread 21%");
    }

    #[test]
    fn d_target_suffix_empty_without_d_or_ratio() {
        assert_eq!(analysis(None, None).d_target_suffix(44, Some((20, 30))), "");
        assert_eq!(analysis(Some(0), Some(2.0)).d_target_suffix(44, None), "");
    }

    // Spec: an Increase recommendation names a P higher than the current P. Target 25 ms
    // (+-3 ms): 30 ms is +20% (moderately slower, +5% step), 33 ms is +32% (significantly
    // slower, +10% step). Low noise is a small D-term HF share; `None` is unknown noise.
    fn increase_target(p: u32, td_ms: f64, hf: Option<f64>) -> Option<u32> {
        let a =
            OptimalPAnalysis::analyze(&[td_ms; 40], p, Some(20), hf, Some(2.0), Some((25.0, 3.0)))
                .expect("analysis");
        match a.recommendation {
            PRecommendation::Increase { conservative_p, .. } => Some(conservative_p),
            _ => None,
        }
    }

    #[test]
    fn small_p_increase_always_names_a_higher_p() {
        let low = Some(DTERM_HF_ENERGY_MODERATE * 0.5);
        for p in [0u32, 1, 5, 9, 10, 19, 20, 21, 60] {
            for (td, hf) in [(30.0, low), (33.0, low), (30.0, None)] {
                let target = increase_target(p, td, hf).expect("increase");
                assert!(target > p, "P {p}, Td {td} ms: Increase to {target}");
            }
        }
    }

    #[test]
    fn small_p_increase_boundaries_at_5_and_10_percent() {
        let low = Some(DTERM_HF_ENERGY_MODERATE * 0.5);
        // +5% (30 ms): 19 -> 19.95 truncates to 19, so the step is raised to 20;
        // 20 -> 21 and 21 -> 22 (22.05) are plain scaling; 0 -> 1 and 1 -> 2 take the minimum step.
        for (p, want) in [(0, 1), (1, 2), (19, 20), (20, 21), (21, 22)] {
            assert_eq!(increase_target(p, 30.0, low), Some(want), "P {p}");
        }
        // +10% (33 ms): 9 -> 9.9 truncates to 9, so 10; 10 -> 11; 20 -> 22.
        for (p, want) in [(9, 10), (10, 11), (20, 22)] {
            assert_eq!(increase_target(p, 33.0, low), Some(want), "P {p}");
        }
    }

    #[test]
    fn within_target_small_p_stays_without_increase() {
        // Spec: Td on target (25 ms), low noise: no increase when the 5% step truncates to the
        // same P (P 10 -> 10.5 -> 10). P 21 -> 22.05 is a real step.
        let low = Some(DTERM_HF_ENERGY_MODERATE * 0.5);
        assert_eq!(increase_target(10, 25.0, low), None);
        assert_eq!(increase_target(19, 25.0, low), None);
    }

    #[test]
    fn raised_p_saturates_at_u32_max() {
        assert_eq!(safe_raised_p(u32::MAX, 1.05), u32::MAX);
    }

    // P 40, D 20, P:D 2.05. Recommended P 41 needs D = 41 / 2.05 = 20, the current D:
    // there is no D change to name, so no suffix (it would read "D≈20 (+0)").
    #[test]
    fn d_target_suffix_is_empty_when_the_target_equals_the_current_d() {
        let a = analysis(Some(20), Some(2.05));
        assert_eq!(a.d_target_suffix(41, None), "");
        // Range 15 to 20: both ends scale by 20 / 20 = 1, so both stay.
        assert_eq!(a.d_target_suffix(41, Some((15, 20))), "");
        // A changed D still prints: P 44 needs D = 44 / 2.05 = 21.46 -> 21.
        assert_eq!(a.d_target_suffix(44, None), ", D≈21 (+1)");
    }
}
