//! `embrig` — the Embrig command-line tool.
//!
//! Subcommands:
//! - `init`      scaffold a vehicle project (vehicle.yaml, DBC, tests)
//! - `simulate`  run the deterministic virtual simulation and print the trace
//! - `test`      run YAML tests against the virtual sim or a CAN interface
//! - `report`    render a JSON suite result to HTML/JSON
//!
//! Exit codes: `0` all pass · `1` test failures · `2` usage/config/load errors.

mod commands;
mod templates;

use std::path::PathBuf;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(
    name = "embrig",
    version,
    about = "Embrig: deterministic hardware-in-the-loop CAN testing"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Scaffold a new vehicle project.
    Init {
        /// Directory to create the project in.
        #[arg(default_value = ".")]
        dir: PathBuf,
        /// Overwrite existing files.
        #[arg(long)]
        force: bool,
    },
    /// Run the virtual simulation and print the bus trace.
    Simulate {
        /// The vehicle.yaml file.
        vehicle: PathBuf,
        /// How long to simulate (e.g. `5s`, `500ms`).
        #[arg(long, default_value = "5s")]
        duration: String,
        /// Print every frame and fault event.
        #[arg(long)]
        verbose: bool,
    },
    /// Run YAML tests against the virtual sim or a CAN interface.
    Test {
        /// The vehicle.yaml file.
        vehicle: PathBuf,
        /// Test files or directories (defaults to `tests` next to vehicle.yaml).
        #[arg(value_name = "TESTS", num_args = 0..)]
        tests: Vec<PathBuf>,
        /// Interface name from vehicle.yaml, or a raw CAN interface name.
        #[arg(long)]
        interface: Option<String>,
        /// Run a loopback check on the interface before the suites.
        #[arg(long)]
        check: bool,
        /// Write a report file (format from --report-format).
        #[arg(long, value_name = "PATH")]
        report: Option<PathBuf>,
        /// Report format: `json` or `html`.
        #[arg(long, default_value = "html")]
        report_format: String,
    },
    /// Render a JSON suite result to HTML or JSON.
    Report {
        /// Input JSON results file.
        input: PathBuf,
        /// Output file (defaults to `report.html`).
        #[arg(long)]
        output: Option<PathBuf>,
        /// Output format: `json` or `html`.
        #[arg(long, default_value = "html")]
        format: String,
    },
}

#[tokio::main]
async fn main() {
    let cli = Cli::parse();
    let code = match commands::run(cli).await {
        Ok(code) => code,
        Err(error) => {
            eprintln!("error: {error:#}");
            2
        }
    };
    std::process::exit(code);
}
