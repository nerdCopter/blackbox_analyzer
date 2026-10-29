//! Non-UTF-8 command-line arguments must produce an error and exit status 1, not a panic (101).
#![cfg(unix)]

use std::ffi::OsString;
use std::fs;
use std::os::unix::ffi::OsStringExt;
use std::path::PathBuf;
use std::process::{Command, Output};

fn scratch_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("bba_non_utf8_{name}_{}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn run(args: Vec<OsString>) -> Output {
    Command::new(env!("CARGO_BIN_EXE_blackbox_analyzer"))
        .args(args)
        .output()
        .unwrap()
}

fn text(out: &Output) -> String {
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

#[test]
fn invalid_input_argument_exits_1_and_valid_input_still_processed() {
    let dir = scratch_dir("input");
    let valid = dir.join("valid_missing.csv");
    let out = run(vec![
        OsString::from_vec(b"bad\xff.csv".to_vec()),
        valid.clone().into_os_string(),
    ]);
    let t = text(&out);
    assert_eq!(out.status.code(), Some(1), "{t}");
    assert!(t.contains("not valid Unicode"), "{t}");
    // The second input was reached: its own (missing-file) error is reported.
    assert!(t.contains("valid_missing.csv"), "{t}");
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn invalid_option_value_exits_1_without_panic() {
    let out = run(vec![
        OsString::from("--output-dir"),
        OsString::from_vec(b"\xff".to_vec()),
        OsString::from("x.csv"),
    ]);
    let t = text(&out);
    assert_eq!(out.status.code(), Some(1), "{t}");
    assert!(t.contains("option value is not valid Unicode"), "{t}");
}

#[test]
fn version_with_invalid_argument_exits_1() {
    let out = run(vec![
        OsString::from("--version"),
        OsString::from_vec(b"bad\xff.csv".to_vec()),
    ]);
    let t = text(&out);
    assert_eq!(out.status.code(), Some(1), "{t}");
    assert!(t.contains("not valid Unicode"), "{t}");
}

#[test]
fn version_alone_exits_0() {
    let out = run(vec![OsString::from("--version")]);
    assert_eq!(out.status.code(), Some(0), "{}", text(&out));
}
