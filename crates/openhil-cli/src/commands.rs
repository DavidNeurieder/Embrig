//! Implementation of the `openhil` subcommands.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use openhil_core::recorder::Record;
use openhil_models::{load_vehicle_config, VehicleConfig};

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
    if !dbc_path.exists() {
        bail!("DBC file `{}` not found", dbc_path.display());
    }
    let duration_us = openhil_test::parse_duration(duration)
        .with_context(|| format!("invalid duration `{duration}`"))?;

    let mut sim = openhil_models::build_simulation(&config, &dbc_path)?;
    sim.run_for(duration_us);

    if verbose {
        for record in &sim.recorder().records {
            match record {
                Record::Frame(frame) => println!("{frame}"),
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
    if !dbc_path.exists() {
        bail!("DBC file `{}` not found", dbc_path.display());
    }
    let vehicle_dir = vehicle
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."))
        .to_path_buf();
    let files = resolve_test_files(test_inputs, &vehicle_dir)?;
    let label = vehicle.display().to_string();

    let target = select_target(&config, &dbc_path, interface)?;

    let suite = match target {
        Target::Virtual(mut target) => {
            let suite = openhil_test::run_suite(&mut *target, &files, &label).await?;
            print_suite(&suite);
            suite
        }
        #[cfg(feature = "socketcan")]
        Target::Hardware(mut target) => {
            let suite = openhil_test::run_suite(&mut target, &files, &label).await?;
            print_suite(&suite);
            suite
        }
    };

    if let Some(report_path) = report {
        openhil_test::write_report(report_path, &suite, report_format)
            .with_context(|| format!("cannot write report `{}`", report_path.display()))?;
        println!("report written to {}", report_path.display());
    }

    Ok(if suite.failed() > 0 { 1 } else { 0 })
}

fn cmd_report(input: &Path, output: Option<&Path>, format: &str) -> Result<i32> {
    let suite = openhil_test::load_json(input)
        .with_context(|| format!("cannot read JSON report `{}`", input.display()))?;
    let output = match output {
        Some(path) => path.to_path_buf(),
        None => PathBuf::from(if format == "json" {
            "report.json"
        } else {
            "report.html"
        }),
    };
    openhil_test::write_report(&output, &suite, format)
        .with_context(|| format!("cannot write report `{}`", output.display()))?;
    println!("report written to {}", output.display());
    Ok(0)
}

/// The target a test suite runs against.
enum Target {
    Virtual(Box<openhil_test::VirtualTarget>),
    #[cfg(feature = "socketcan")]
    Hardware(openhil_test::target::HardwareTarget),
}

fn select_target(
    config: &VehicleConfig,
    dbc_path: &Path,
    interface: Option<&str>,
) -> Result<Target> {
    let kind = match interface {
        None => "virtual".to_string(),
        Some(name) => config
            .interfaces
            .iter()
            .find(|i| i.name == name)
            .map(|i| i.kind.clone())
            .ok_or_else(|| anyhow::anyhow!("interface `{name}` not found in vehicle.yaml"))?,
    };

    match kind.as_str() {
        "virtual" => {
            let target = openhil_test::VirtualTarget::new(config, dbc_path).with_context(|| {
                format!(
                    "cannot build virtual simulation from `{}`",
                    dbc_path.display()
                )
            })?;
            Ok(Target::Virtual(Box::new(target)))
        }
        "socketcan" => {
            #[cfg(feature = "socketcan")]
            {
                let iface_name = interface
                    .and_then(|name| {
                        config
                            .interfaces
                            .iter()
                            .find(|i| i.name == name)
                            .and_then(|i| i.interface.clone())
                    })
                    .unwrap_or_else(|| "vcan0".to_string());
                let text = fs::read_to_string(dbc_path)
                    .with_context(|| format!("cannot read `{}`", dbc_path.display()))?;
                let network = openhil_dbc::parse(&text)
                    .with_context(|| format!("invalid DBC `{}`", dbc_path.display()))?;
                let target = openhil_test::target::HardwareTarget::new(&iface_name, network)
                    .with_context(|| format!("cannot open CAN interface `{iface_name}`"))?;
                Ok(Target::Hardware(target))
            }
            #[cfg(not(feature = "socketcan"))]
            {
                let _ = config;
                bail!("this build has no socketcan support; rebuild with `--features socketcan`")
            }
        }
        other => bail!("unknown interface type `{other}` (use virtual or socketcan)"),
    }
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
            let mut files: Vec<PathBuf> = fs::read_dir(&path)
                .with_context(|| format!("cannot read `{}`", path.display()))?
                .filter_map(|entry| entry.ok().map(|e| e.path()))
                .filter(|p| {
                    p.extension()
                        .is_some_and(|ext| ext == "yaml" || ext == "yml")
                })
                .collect();
            files.sort();
            out.extend(files);
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

fn print_suite(suite: &openhil_test::SuiteResult) {
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
