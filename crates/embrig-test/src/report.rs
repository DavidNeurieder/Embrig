//! Test result aggregation and JSON/HTML reports.

use std::fmt::Write as _;
use std::path::Path;

use serde::{Deserialize, Serialize};

/// The outcome of a single test.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    pub name: String,
    pub passed: bool,
    pub steps: usize,
    /// Simulation time consumed by the test (µs).
    pub duration_us: u64,
    #[serde(default)]
    pub failures: Vec<String>,
}

/// The outcome of a whole test file (suite).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuiteResult {
    /// Display path of the test file, or the suite name.
    pub file: String,
    pub duration_us: u64,
    pub tests: Vec<TestResult>,
}

impl SuiteResult {
    pub fn passed(&self) -> usize {
        self.tests.iter().filter(|t| t.passed).count()
    }

    pub fn failed(&self) -> usize {
        self.tests.len() - self.passed()
    }
}

/// Print a suite result to stdout in the standard `PASS`/`FAIL` format.
pub fn print_suite(suite: &SuiteResult) {
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

/// Serialize a suite result as pretty JSON.
pub fn json(suite: &SuiteResult) -> String {
    serde_json::to_string_pretty(suite).unwrap_or_else(|e| e.to_string())
}

/// Load a suite result previously written with [`json`].
pub fn load_json(path: &Path) -> std::io::Result<SuiteResult> {
    let text = std::fs::read_to_string(path)?;
    serde_json::from_str(&text).map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
}

/// Escape HTML special characters for report text.
fn escape(input: &str) -> String {
    let mut out = String::with_capacity(input.len());
    for c in input.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}

/// Serialize a suite result as a self-contained HTML page.
pub fn html(suite: &SuiteResult) -> String {
    let mut out = String::new();
    out.push_str("<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n");
    out.push_str("<meta charset=\"utf-8\">\n");
    out.push_str(&format!(
        "<title>Embrig report — {}</title>\n",
        escape(&suite.file)
    ));
    out.push_str(
        "<style>body{font-family:system-ui,sans-serif;margin:2rem;color:#222}\
         table{border-collapse:collapse;width:100%;margin-top:1rem}\
         th,td{text-align:left;padding:.4rem .6rem;border-bottom:1px solid #ddd}\
         .pass{color:#0a7a33}.fail{color:#b00}.mono{font-family:ui-monospace,monospace}\
         .summary{font-size:1.1rem}.failures{color:#b00;font-size:.9rem;margin:.2rem 0}\
         code{background:#f4f4f4;padding:.1rem .3rem}</style>\n",
    );
    out.push_str("</head>\n<body>\n");
    out.push_str(&format!(
        "<h1>Embrig report — <span class=\"mono\">{}</span></h1>\n",
        escape(&suite.file)
    ));
    out.push_str(&format!(
        "<p class=\"summary\">{} passed, {} failed · {:.0} ms</p>\n",
        suite.passed(),
        suite.failed(),
        suite.duration_us as f64 / 1000.0
    ));
    out.push_str(
        "<table>\n<tr><th>Test</th><th>Result</th><th>Duration</th><th>Failures</th></tr>\n",
    );
    for test in &suite.tests {
        let class = if test.passed { "pass" } else { "fail" };
        out.push_str("<tr>");
        out.push_str(&format!(
            "<td class=\"mono\">{}</td><td class=\"{}\">{}</td><td>{:.1} ms</td><td>",
            escape(&test.name),
            class,
            if test.passed { "PASS" } else { "FAIL" },
            test.duration_us as f64 / 1000.0
        ));
        if test.failures.is_empty() {
            out.push('—');
        } else {
            for failure in &test.failures {
                let _ = writeln!(
                    out,
                    "<div class=\"failures\"><code>{}</code></div>",
                    escape(failure)
                );
            }
        }
        out.push_str("</td></tr>\n");
    }
    out.push_str("</table>\n</body>\n</html>\n");
    out
}

/// Write a suite report to `path` in `format` (`json` or `html`).
pub fn write_report(path: &Path, suite: &SuiteResult, format: &str) -> std::io::Result<()> {
    let body = match format {
        "json" => json(suite),
        "html" => html(suite),
        other => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("unknown report format `{other}` (use json or html)"),
            ))
        }
    };
    std::fs::write(path, body)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn suite() -> SuiteResult {
        SuiteResult {
            file: "tests/powertrain.yaml".into(),
            duration_us: 1_500_000,
            tests: vec![
                TestResult {
                    name: "ok".into(),
                    passed: true,
                    steps: 3,
                    duration_us: 500_000,
                    failures: vec![],
                },
                TestResult {
                    name: "bad".into(),
                    passed: false,
                    steps: 2,
                    duration_us: 1_000_000,
                    failures: vec!["0x220.motor_enable expected < true, got false".into()],
                },
            ],
        }
    }

    #[test]
    fn json_round_trips() {
        let s = json(&suite());
        let parsed: SuiteResult = serde_json::from_str(&s).unwrap();
        assert_eq!(parsed.passed(), 1);
        assert_eq!(parsed.failed(), 1);
    }

    #[test]
    fn html_counts() {
        let h = html(&suite());
        assert!(h.contains("1 passed, 1 failed"));
        assert!(h.contains("PASS"));
        assert!(h.contains("FAIL"));
        assert!(h.contains("&lt;"));
    }

    #[test]
    fn escape_handles_special_chars() {
        assert_eq!(escape("<a & b>\"'\n"), "&lt;a &amp; b&gt;&quot;&#39;\n");
    }
}
