//! Black-box tests for the `embrig` binary: exit codes, scaffolding and
//! report files (per `IMPLEMENTATION_PLAN.md` verification step 6).

use std::path::PathBuf;
use std::process::{Command, Output};

const BIN: &str = env!("CARGO_BIN_EXE_embrig");

fn tmpdir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("embrig-cli-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn run(args: &[&str]) -> Output {
    Command::new(BIN).args(args).output().unwrap()
}

fn run_cwd(dir: &std::path::Path, args: &[&str]) -> Output {
    Command::new(BIN)
        .args(args)
        .current_dir(dir)
        .output()
        .unwrap()
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
fn init_scaffolds_expected_content() {
    let dir = tmpdir("content");
    run(&["init", dir.to_str().unwrap()]);

    let vehicle = std::fs::read_to_string(dir.join("vehicle.yaml")).unwrap();
    assert!(vehicle.contains("name: ev-powertrain"));
    assert!(
        vehicle.contains("type: socketcan"),
        "hardware interface present"
    );

    let dbc = std::fs::read_to_string(dir.join("powertrain.dbc")).unwrap();
    assert!(dbc.contains("BO_ 544 MotorEnable"));
    assert!(dbc.contains("VAL_ 560 state 0 \"OFF\" 1 \"READY\" 2 \"RUNNING\" 3 \"SAFE\""));

    let nominal = std::fs::read_to_string(dir.join("tests/nominal_conditions.yaml")).unwrap();
    assert!(nominal.contains("nominal_conditions_enable_motor"));
    let overvoltage = std::fs::read_to_string(dir.join("tests/overvoltage.yaml")).unwrap();
    assert!(overvoltage.contains("SAFE"));
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
fn simulate_verbose_prints_frame_trace() {
    let dir = tmpdir("verbose");
    run(&["init", dir.to_str().unwrap()]);
    let out = run(&[
        "simulate",
        dir.join("vehicle.yaml").to_str().unwrap(),
        "--duration",
        "200ms",
        "--verbose",
    ]);
    assert!(out.status.success());
    let out = stdout(&out);
    assert!(out.contains("0x100"), "missing battery frames: {out}");
    assert!(out.contains("0x220"), "missing motor-enable frames: {out}");
    assert!(out.contains("simulated 200 ms"));
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
    let parsed: embrig_test::SuiteResult =
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

#[test]
fn test_with_explicit_file_list() {
    let dir = tmpdir("explicit");
    run(&["init", dir.to_str().unwrap()]);

    // Explicit files, resolved relative to the vehicle dir.
    let files = run_cwd(
        &dir,
        &[
            "test",
            dir.join("vehicle.yaml").to_str().unwrap(),
            "tests/overvoltage.yaml",
            "tests/nominal_conditions.yaml",
        ],
    );
    assert_eq!(files.status.code(), Some(0), "stderr: {}", stderr(&files));
    let out = stdout(&files);
    assert!(out.contains("2 passed"), "got: {out}");
    assert!(out.contains("overvoltage_disables_motor"));

    // A directory input expands to all YAML files inside it.
    let dir_input = run_cwd(
        &dir,
        &["test", dir.join("vehicle.yaml").to_str().unwrap(), "tests"],
    );
    assert_eq!(dir_input.status.code(), Some(0));
    assert!(stdout(&dir_input).contains("5 passed"));
}

#[test]
fn test_rejects_unknown_interface_name() {
    let dir = tmpdir("iface");
    run(&["init", dir.to_str().unwrap()]);
    let out = run(&[
        "test",
        dir.join("vehicle.yaml").to_str().unwrap(),
        "--interface",
        "can9",
    ]);
    assert_eq!(out.status.code(), Some(2), "stderr: {}", stderr(&out));
    assert!(stderr(&out).contains("not found in vehicle.yaml"));
}

#[test]
fn test_sil_interface_points_to_embrig_sil() {
    let dir = tmpdir("sil-iface");
    run(&["init", dir.to_str().unwrap()]);
    let vehicle = dir.join("vehicle.yaml");
    let mut text = std::fs::read_to_string(&vehicle).unwrap();
    text.push_str("  - name: sil\n    type: sil\n");
    std::fs::write(&vehicle, text).unwrap();
    let out = run(&["test", vehicle.to_str().unwrap(), "--interface", "sil"]);
    assert_eq!(out.status.code(), Some(2), "stderr: {}", stderr(&out));
    assert!(
        stderr(&out).contains("embrig-sil"),
        "stderr should point to embrig-sil: {}",
        stderr(&out)
    );
}

#[test]
fn report_defaults_and_rejects_bad_format() {
    let dir = tmpdir("report2");
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
    assert!(tested.status.success());

    // `report` with no --format/--output defaults to report.html in the cwd.
    let rendered = run_cwd(&dir, &["report", report_json.to_str().unwrap()]);
    assert!(rendered.status.success(), "stderr: {}", stderr(&rendered));
    let body = std::fs::read_to_string(dir.join("report.html")).unwrap();
    assert!(body.contains("<html"));
    assert!(body.contains("5 passed"));

    // `--format json --output` writes a parseable suite result.
    let copy = dir.join("copy.json");
    let rendered_json = run_cwd(
        &dir,
        &[
            "report",
            report_json.to_str().unwrap(),
            "--format",
            "json",
            "--output",
            copy.to_str().unwrap(),
        ],
    );
    assert!(rendered_json.status.success());
    let parsed: embrig_test::SuiteResult =
        serde_json::from_str(&std::fs::read_to_string(&copy).unwrap()).unwrap();
    assert_eq!(parsed.failed(), 0);

    // Unknown formats are usage errors (exit 2), for both subcommands.
    let bad = run(&["report", report_json.to_str().unwrap(), "--format", "pdf"]);
    assert_eq!(bad.status.code(), Some(2));
    assert!(stderr(&bad).contains("unknown report format"));

    let bad_test = run(&[
        "test",
        &vehicle,
        "--report",
        dir.join("bad.json").to_str().unwrap(),
        "--report-format",
        "pdf",
    ]);
    assert_eq!(bad_test.status.code(), Some(2));
    assert!(stderr(&bad_test).contains("unknown report format"));
}
