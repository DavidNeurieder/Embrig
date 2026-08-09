//! Black-box tests for the `openhil` binary: exit codes, scaffolding and
//! report files (per `IMPLEMENTATION_PLAN.md` verification step 6).

use std::path::PathBuf;
use std::process::{Command, Output};

const BIN: &str = env!("CARGO_BIN_EXE_openhil");

fn tmpdir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("openhil-cli-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn run(args: &[&str]) -> Output {
    Command::new(BIN).args(args).output().unwrap()
}

fn stdout(out: &Output) -> String {
    String::from_utf8_lossy(&out.stdout).into_owned()
}

fn stderr(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

#[test]
fn init_scaffolds_project_and_rejects_overwrite() {
    let dir = tmpdir("init");
    let out = run(&["init", dir.to_str().unwrap()]);
    assert!(out.status.success());
    for file in [
        "vehicle.yaml",
        "powertrain.dbc",
        "tests/nominal_conditions.yaml",
        "tests/overvoltage.yaml",
    ] {
        assert!(dir.join(file).exists(), "missing {file}");
    }

    // Re-running without --force is a usage error (exit 2).
    let again = run(&["init", dir.to_str().unwrap()]);
    assert_eq!(again.status.code(), Some(2));
    assert!(stderr(&again).contains("already exists"));

    // --force overwrites cleanly.
    let forced = run(&["init", dir.to_str().unwrap(), "--force"]);
    assert!(forced.status.success());
}

#[test]
fn test_exit_codes_pass_fail_and_usage() {
    let dir = tmpdir("codes");
    run(&["init", dir.to_str().unwrap()]);

    // A passing suite exits 0.
    let pass = run(&["test", dir.join("vehicle.yaml").to_str().unwrap()]);
    assert_eq!(pass.status.code(), Some(0), "stderr: {}", stderr(&pass));
    assert!(stdout(&pass).contains("5 passed"));

    // A failing test file exits 1 and explains the failure.
    let failing = "name: fails\ntimeout: 5s\nsteps:\n  - expect: { id: 0x999, present: true, within: 200ms }\n";
    let fail_path = dir.join("tests/fails.yaml");
    std::fs::write(&fail_path, failing).unwrap();
    let fail = run(&[
        "test",
        dir.join("vehicle.yaml").to_str().unwrap(),
        fail_path.to_str().unwrap(),
    ]);
    assert_eq!(fail.status.code(), Some(1));
    assert!(stdout(&fail).contains("0 passed, 1 failed"));

    // A missing vehicle file is a usage/config error (exit 2).
    let missing = run(&["test", dir.join("nope.yaml").to_str().unwrap()]);
    assert_eq!(missing.status.code(), Some(2));
}

#[test]
fn simulate_runs_and_reports_duration() {
    let dir = tmpdir("simulate");
    run(&["init", dir.to_str().unwrap()]);
    let out = run(&[
        "simulate",
        dir.join("vehicle.yaml").to_str().unwrap(),
        "--duration",
        "200ms",
    ]);
    assert!(out.status.success());
    assert!(stdout(&out).contains("simulated 200 ms"));
}

#[test]
fn report_writes_json_then_renders_html() {
    let dir = tmpdir("report");
    run(&["init", dir.to_str().unwrap()]);
    let vehicle = dir.join("vehicle.yaml").to_str().unwrap().to_string();

    let report_json = dir.join("results.json");
    let tested = run(&[
        "test",
        &vehicle,
        "--report",
        report_json.to_str().unwrap(),
        "--report-format",
        "json",
    ]);
    assert!(tested.status.success(), "stderr: {}", stderr(&tested));
    assert!(report_json.exists());

    // The stored JSON is a valid suite result.
    let parsed: openhil_test::SuiteResult =
        serde_json::from_str(&std::fs::read_to_string(&report_json).unwrap()).unwrap();
    assert_eq!(parsed.failed(), 0);

    // The `report` subcommand renders it to HTML.
    let html = dir.join("results.html");
    let rendered = run(&[
        "report",
        report_json.to_str().unwrap(),
        "--format",
        "html",
        "--output",
        html.to_str().unwrap(),
    ]);
    assert!(rendered.status.success());
    let body = std::fs::read_to_string(&html).unwrap();
    assert!(body.contains("<html"));
    assert!(body.contains("5 passed"));
}
