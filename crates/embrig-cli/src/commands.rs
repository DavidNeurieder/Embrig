//! Implementation of the `embrig` subcommands.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use embrig_core::network::CanSimExt;
use embrig_core::recorder::Record;
use embrig_models::{load_vehicle_config, VehicleConfig};

use crate::{templates, Cli, Command};

/// Dispatch a parsed command; returns the process exit code.
pub async fn run(cli: Cli) -> Result<i32> {
    match cli.command {
        Command::Init { dir, force } => cmd_init(&dir, force),
        Command::Simulate {
            vehicle,
            duration,
            verbose,
        } => cmd_simulate(&vehicle, &duration, verbose),
        Command::Test {
            vehicle,
            tests,
            interface,
            report,
            report_format,
        } => {
            cmd_test(
                &vehicle,
                &tests,
                interface.as_deref(),
                report.as_deref(),
                &report_format,
            )
            .await
        }
        Command::Report {
            input,
            output,
            format,
        } => cmd_report(&input, output.as_deref(), &format),
    }
}

fn cmd_init(dir: &Path, force: bool) -> Result<i32> {
    fs::create_dir_all(dir).with_context(|| format!("cannot create `{}`", dir.display()))?;
    fs::create_dir_all(dir.join("tests"))
        .with_context(|| format!("cannot create `{}`", dir.join("tests").display()))?;

    let files: Vec<(PathBuf, &str)> = vec![
        (dir.join("vehicle.yaml"), templates::VEHICLE_YAML),
        (dir.join("powertrain.dbc"), templates::POWERTRAIN_DBC),
        (
            dir.join("tests/nominal_conditions.yaml"),
            templates::TEST_NOMINAL,
        ),
        (
            dir.join("tests/overvoltage.yaml"),
            templates::TEST_OVERVOLTAGE,
        ),
        (dir.join("tests/brake_safety.yaml"), templates::TEST_BRAKE),
        (
            dir.join("tests/charger_fault.yaml"),
            templates::TEST_CHARGER_FAULT,
        ),
        (dir.join("tests/bus_frames.yaml"), templates::TEST_PRESENT),
    ];

    for (path, content) in &files {
        if path.exists() && !force {
            bail!(
                "`{}` already exists (use --force to overwrite)",
                path.display()
            );
        }
        let _ = content;
    }
    for (path, content) in &files {
        fs::write(path, content).with_context(|| format!("cannot write `{}`", path.display()))?;
        println!("created {}", path.display());
    }
    Ok(0)
}

fn cmd_simulate(vehicle: &Path, duration: &str, verbose: bool) -> Result<i32> {
    let (config, dbc_path) = load_vehicle_config(vehicle)
        .with_context(|| format!("cannot load `{}`", vehicle.display()))?;
    if config.dbc.is_empty() {
        bail!(
            "`simulate` runs the CAN simulation and needs a DBC; `{}` has no `dbc` \
             (Ethernet traffic is exercised by the UDP test targets)",
            vehicle.display()
        );
    }
    if !dbc_path.exists() {
        bail!("DBC file `{}` not found", dbc_path.display());
    }
    let duration_us = embrig_test::parse_duration(duration)
        .with_context(|| format!("invalid duration `{duration}`"))?;

    let mut sim = embrig_models::build_simulation(&config, &dbc_path)?;
    sim.run_for(duration_us);

    if verbose {
        for record in &sim.recorder().records {
            match record {
                Record::Message(frame) => println!("{frame}"),
                Record::Event {
                    ts,
                    source,
                    message,
                } => {
                    println!("{ts:>12} [{source}] {message}")
                }
            }
        }
    } else {
        for (id, count) in sim.frame_counts() {
            println!("0x{id:03X}: {count} frames");
        }
    }
    println!("simulated {} ms", sim.time() / 1000);
    Ok(0)
}

async fn cmd_test(
    vehicle: &Path,
    test_inputs: &[PathBuf],
    interface: Option<&str>,
    report: Option<&Path>,
    report_format: &str,
) -> Result<i32> {
    let (config, dbc_path) = load_vehicle_config(vehicle)
        .with_context(|| format!("cannot load `{}`", vehicle.display()))?;
    if !config.dbc.is_empty() && !dbc_path.exists() {
        bail!("DBC file `{}` not found", dbc_path.display());
    }
    let vehicle_dir = vehicle
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."))
        .to_path_buf();
    // The default test location is `tests` next to vehicle.yaml, resolved in
    // code (not as a CLI default) so a stray `tests/` dir in the cwd never
    // shadows it.
    let files = if test_inputs.is_empty() {
        let default_dir = vehicle_dir.join("tests");
        if !default_dir.is_dir() {
            bail!(
                "no test files given and default `{}` does not exist",
                default_dir.display()
            );
        }
        read_yaml_files(&default_dir)?
    } else {
        resolve_test_files(test_inputs, &vehicle_dir)?
    };
    let label = vehicle.display().to_string();

    let kind = interface_kind(&config, interface)?;
    if kind == "sil" {
        bail!(
            "interface `sil` is software-in-the-loop, which the CLI does not execute; \
             use the `embrig-sil` crate instead \
             (`cargo run --example sil_firmware --package embrig-sil`)"
        );
    }

    let input = embrig_test::ProtocolInput {
        config: &config,
        dbc_path: &dbc_path,
        vehicle_dir: &vehicle_dir,
        interface,
    };
    let registry = embrig_test::ProtocolRegistry::default();
    let mut target = registry
        .build(&kind, &input)
        .with_context(|| format!("cannot build `{kind}` target"))?;

    let suite = embrig_test::run_suite(&mut *target, &files, &label).await?;
    print_suite(&suite);

    if let Some(report_path) = report {
        embrig_test::write_report(report_path, &suite, report_format)
            .with_context(|| format!("cannot write report `{}`", report_path.display()))?;
        println!("report written to {}", report_path.display());
    }

    Ok(if suite.failed() > 0 { 1 } else { 0 })
}

/// Resolve the interface kind from the CLI flag (or the virtual default).
fn interface_kind(config: &VehicleConfig, interface: Option<&str>) -> Result<String> {
    match interface {
        None => Ok("virtual".to_string()),
        Some(name) => config
            .interfaces
            .iter()
            .find(|i| i.name == name)
            .map(|i| i.kind.clone())
            .ok_or_else(|| anyhow::anyhow!("interface `{name}` not found in vehicle.yaml")),
    }
}

fn cmd_report(input: &Path, output: Option<&Path>, format: &str) -> Result<i32> {
    let suite = embrig_test::load_json(input)
        .with_context(|| format!("cannot read JSON report `{}`", input.display()))?;
    let output = match output {
        Some(path) => path.to_path_buf(),
        None => PathBuf::from(if format == "json" {
            "report.json"
        } else {
            "report.html"
        }),
    };
    embrig_test::write_report(&output, &suite, format)
        .with_context(|| format!("cannot write report `{}`", output.display()))?;
    println!("report written to {}", output.display());
    Ok(0)
}

/// Expand test inputs (files or directories) into a sorted, deduplicated list.
fn resolve_test_files(inputs: &[PathBuf], vehicle_dir: &Path) -> Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    for input in inputs {
        let path = if input.exists() {
            input.clone()
        } else if vehicle_dir.join(input).exists() {
            vehicle_dir.join(input)
        } else {
            bail!(
                "test path `{}` not found (relative to cwd or `{}`)",
                input.display(),
                vehicle_dir.display()
            );
        };
        if path.is_dir() {
            out.extend(read_yaml_files(&path)?);
        } else {
            out.push(path);
        }
    }
    out.sort();
    out.dedup();
    if out.is_empty() {
        bail!("no test files found");
    }
    Ok(out)
}

/// All `*.yaml`/`*.yml` files inside a directory, sorted.
fn read_yaml_files(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut files: Vec<PathBuf> = fs::read_dir(dir)
        .with_context(|| format!("cannot read `{}`", dir.display()))?
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|p| {
            p.extension()
                .is_some_and(|ext| ext == "yaml" || ext == "yml")
        })
        .collect();
    files.sort();
    Ok(files)
}

fn print_suite(suite: &embrig_test::SuiteResult) {
    for test in &suite.tests {
        let status = if test.passed { "PASS" } else { "FAIL" };
        println!(
            "{status}  {}  ({:.0} ms)",
            test.name,
            test.duration_us as f64 / 1000.0
        );
        for failure in &test.failures {
            println!("       {failure}");
        }
    }
    println!("{} passed, {} failed", suite.passed(), suite.failed());
}
