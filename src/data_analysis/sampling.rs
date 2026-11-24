// src/data_analysis/sampling.rs

/// True when the `len` samples starting at `start` span more time than uniform sampling at
/// `sample_rate` allows, by more than `tolerance` times (`(len - 1) / sample_rate`). That is
/// the signature of a dropped-frame gap or log pause inside the window, whose samples then
/// are not uniformly spaced. A window shorter than two samples or past the end of `times`
/// has no span and returns `false`.
pub fn window_spans_gap(
    times: &[f64],
    start: usize,
    len: usize,
    sample_rate: f64,
    tolerance: f64,
) -> bool {
    if len < 2 || start + len > times.len() {
        return false;
    }
    let expected_span_s = (len - 1) as f64 / sample_rate;
    times[start + len - 1] - times[start] > expected_span_s * tolerance
}

#[cfg(test)]
mod tests {
    use super::*;

    // Spec: 11 samples at 100 Hz span 0.1 s; with tolerance 1.5 the limit is 0.15 s.
    #[test]
    fn span_limit_is_the_tolerance_times_the_uniform_span() {
        let uniform: Vec<f64> = (0..11).map(|i| f64::from(i) * 0.01).collect();
        assert!(!window_spans_gap(&uniform, 0, 11, 100.0, 1.5));
        let mut at_limit = uniform.clone();
        at_limit[10] = 0.15;
        assert!(!window_spans_gap(&at_limit, 0, 11, 100.0, 1.5));
        at_limit[10] = 0.151;
        assert!(window_spans_gap(&at_limit, 0, 11, 100.0, 1.5));
    }

    // A gap outside the checked window does not count.
    #[test]
    fn only_the_window_is_checked() {
        let mut times: Vec<f64> = (0..30).map(|i| f64::from(i) * 0.01).collect();
        for t in &mut times[20..] {
            *t += 5.0;
        }
        assert!(!window_spans_gap(&times, 0, 10, 100.0, 1.5));
        assert!(window_spans_gap(&times, 15, 10, 100.0, 1.5));
    }

    // Degenerate windows have no span.
    #[test]
    fn degenerate_windows_have_no_gap() {
        let times = [0.0, 100.0];
        assert!(!window_spans_gap(&times, 0, 0, 100.0, 1.5));
        assert!(!window_spans_gap(&times, 0, 1, 100.0, 1.5));
        assert!(!window_spans_gap(&times, 1, 2, 100.0, 1.5));
        assert!(window_spans_gap(&times, 0, 2, 100.0, 1.5));
        assert!(!window_spans_gap(&[], 0, 2, 100.0, 1.5));
    }
}
