//! Local optimization of a trajectory by perturbing waypoint poses within
//! user-defined bounds.
//!
//! Each waypoint can carry an optimization bound (`dx`, `dy`, `dtheta`), the
//! maximum amount its pose may be perturbed in each degree of freedom. This
//! module performs a coordinate descent: it repeatedly tries perturbing each
//! waypoint by ± its bound, regenerates the trajectory, and keeps perturbations
//! that reduce the total trajectory time. It converges when a full sweep over
//! every waypoint produces no improvement, i.e. a local minimum in time.
//!
//! Only waypoints whose translation/heading are *fixed* constrain the generated
//! path (the generator turns them into `pose_wpt`/`translation_wpt` equality
//! constraints), so only they are perturbed within their bounds. Unfixed
//! waypoints are free initial guesses the solver places on its own, so
//! perturbing them would have no effect on the trajectory.

use tokio::sync::oneshot;

use super::generate::{generate, LocalProgressUpdate};
use crate::spec::trajectory::{DriveType, Sample, TrajectoryFile};
use crate::spec::{Expr, SnapshottableType, project::ProjectFile};
use crate::{ChoreoError, ChoreoResult, ResultExt};

/// The maximum number of coordinate-descent sweeps to perform. Each sweep
/// evaluates up to 6 perturbations per waypoint, so this bounds the total
/// number of trajectory generations.
const MAX_SWEEPS: usize = 20;

fn round(input: f64) -> f64 {
    let factor = 100_000.0;
    let result = (input * factor).round() / factor;
    if result == -0.0 { 0.0 } else { result }
}

/// The total time of a trajectory, i.e. the timestamp of its final sample.
fn total_time(file: &TrajectoryFile) -> f64 {
    file.trajectory
        .samples
        .last()
        .map_or(0.0, Sample::t)
}

fn send_progress(update: LocalProgressUpdate, handle: i64) {
    if let Some(tx) = super::generate::PROGRESS_SENDER_LOCK.get() {
        let _ = tx.send(update.handled(handle)).trace_warn();
    }
}

fn emit_diagnostic(message: String, handle: i64) {
    send_progress(LocalProgressUpdate::DiagnosticText { update: message }, handle);
}

/// Emit the samples of `file` so the frontend can display the current best
/// trajectory while the optimization runs.
fn emit_trajectory(file: &TrajectoryFile, handle: i64) {
    match file.trajectory.sample_type {
        Some(DriveType::Swerve) => send_progress(
            LocalProgressUpdate::SwerveTrajectory {
                update: file.trajectory.samples.clone(),
            },
            handle,
        ),
        Some(DriveType::Differential) => send_progress(
            LocalProgressUpdate::DifferentialTrajectory {
                update: file.trajectory.samples.clone(),
            },
            handle,
        ),
        None => {}
    }
}

/// A regex matching a bare numeric expression with a trailing unit, e.g.
/// `1.5 m`, `-2 in`, or `0 deg`. Used to preserve the unit when perturbing an
/// expression.
fn literal_unit(exp: &str) -> Option<String> {
    let re = regex::Regex::new(r"^[+-]?[0-9]*\.?[0-9]+\s*([^\s0-9.+-]+)$").ok()?;
    re.captures(exp.trim())
        .map(|caps| caps.get(1).unwrap().as_str().to_string())
}

/// Return a new `Expr` equal to `expr` shifted by `delta`, keeping the
/// expression string consistent with the new value so it can be parsed by the
/// frontend (which evaluates the expression rather than trusting `val`).
fn perturb_expr(expr: &Expr, delta: f64, fallback_unit: &str) -> Expr {
    let new_val = round(expr.val + delta);
    let unit = literal_unit(&expr.exp).unwrap_or_else(|| fallback_unit.to_string());
    Expr::new(&format!("{new_val} {unit}"), new_val)
}

/// Perturb the pose of waypoint `idx` in `file`'s parameters by (`dx`, `dy`,
/// `dtheta`) and return the new file.
fn perturb_waypoint(file: &TrajectoryFile, idx: usize, dx: f64, dy: f64, dtheta: f64) -> TrajectoryFile {
    let mut out = file.clone();
    let waypoint = &mut out.params.waypoints[idx];
    waypoint.x = perturb_expr(&waypoint.x, dx, "m");
    waypoint.y = perturb_expr(&waypoint.y, dy, "m");
    waypoint.heading = perturb_expr(&waypoint.heading, dtheta, "rad");
    out
}

/// A single degree of freedom of a waypoint, describing how it may be
/// perturbed during optimization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dof {
    Dx,
    Dy,
    Dtheta,
}

/// Which degrees of freedom of `waypoint` are optimizable, given its bounds.
///
/// Only fixed waypoints constrain the generated path (the generator turns
/// `fix_translation`/`fix_heading` into `pose_wpt`/`translation_wpt` equality
/// constraints), so only they are perturbed within their bounds. Unfixed
/// waypoints are free initial guesses the solver places on its own; perturbing
/// them would not change the trajectory.
fn optimizable_dofs(file: &TrajectoryFile, idx: usize) -> Vec<(Dof, f64)> {
    let waypoint = &file.params.waypoints[idx];
    let mut out = Vec::new();
    if waypoint.fix_translation {
        let dx = waypoint.dx.snapshot();
        let dy = waypoint.dy.snapshot();
        if dx != 0.0 {
            out.push((Dof::Dx, dx));
        }
        if dy != 0.0 {
            out.push((Dof::Dy, dy));
        }
    }
    if waypoint.fix_heading {
        let dtheta = waypoint.dtheta.snapshot();
        if dtheta != 0.0 {
            out.push((Dof::Dtheta, dtheta));
        }
    }
    out
}

/// Perturb `file`'s waypoint `idx` by `+amount` or `-amount` in the given
/// degree of freedom.
fn candidate(file: &TrajectoryFile, idx: usize, dof: Dof, amount: f64, sign: f64) -> TrajectoryFile {
    let (dx, dy, dtheta) = match dof {
        Dof::Dx => (amount * sign, 0.0, 0.0),
        Dof::Dy => (0.0, amount * sign, 0.0),
        Dof::Dtheta => (0.0, 0.0, amount * sign),
    };
    perturb_waypoint(file, idx, dx, dy, dtheta)
}

/// Optimize a trajectory by perturbing each waypoint's pose within its
/// per-waypoint bounds (`dx`, `dy`, `dtheta`) to find a local minimum in total
/// trajectory time.
///
/// The waypoints of `trajectory_file.params` are perturbed, and each candidate
/// is regenerated in-process. Progress is streamed to the frontend through the
/// progress sender. Generation is canceled when `cancel` is triggered.
///
/// # Errors
/// - [`ChoreoError`] if generation of the initial trajectory fails.
/// - [`ChoreoError::RemoteGenerationError`] if the optimization was canceled.
pub fn optimize(
    project: ProjectFile,
    trajectory_file: TrajectoryFile,
    handle: i64,
    cancel: oneshot::Receiver<()>,
) -> ChoreoResult<TrajectoryFile> {
    let mut cancel = cancel;

    // Establish a baseline by regenerating the given trajectory. This also
    // handles stale trajectories.
    let mut best = generate(project.clone(), trajectory_file, handle)?;
    let mut best_time = total_time(&best);
    emit_diagnostic(
        format!("Optimization started. Initial time: {best_time:.4} s"),
        handle,
    );
    emit_trajectory(&best, handle);

    for sweep in 0..MAX_SWEEPS {
        let mut sweep_improved = false;
        let mut current = best.clone();

        for idx in 0..current.params.waypoints.len() {
            // Check cancellation between generations.
            if cancel.try_recv().is_ok() {
                return Err(ChoreoError::remote(ChoreoError::Subprocess(
                    "Optimization canceled".to_string(),
                )));
            }

            let dofs = optimizable_dofs(&current, idx);
            if dofs.is_empty() {
                continue;
            }

            for (dof, amount) in dofs {
                for sign in [1.0, -1.0] {
                    let candidate_file = candidate(&current, idx, dof, amount, sign);
                    match generate(project.clone(), candidate_file, handle) {
                        Ok(result) => {
                            let time = total_time(&result);
                            if time < best_time {
                                best_time = time;
                                best = result;
                                current = best.clone();
                                sweep_improved = true;
                                emit_diagnostic(
                                    format!(
                                        "Optimization sweep {sweep}: waypoint {} {} moved, new best time: {best_time:.4} s",
                                        idx + 1,
                                        match dof {
                                            Dof::Dx => "x",
                                            Dof::Dy => "y",
                                            Dof::Dtheta => "theta",
                                        }
                                    ),
                                    handle,
                                );
                                emit_trajectory(&best, handle);
                            }
                        }
                        Err(e) => {
                            // Infeasible perturbations (e.g. colliding with a
                            // keep-out zone) are skipped, not fatal.
                            tracing::warn!(
                                "Optimization: perturbing waypoint {} failed: {e}",
                                idx + 1
                            );
                        }
                    }
                }
            }
        }

        if !sweep_improved {
            break;
        }
    }

    emit_diagnostic(format!("Optimization complete. Final time: {best_time:.4} s"), handle);
    emit_trajectory(&best, handle);
    Ok(best)
}

#[cfg(test)]
mod tests {
    use crate::spec::trajectory::{Parameters, Waypoint};

    use super::*;

    fn test_trajectory() -> TrajectoryFile {
        TrajectoryFile {
            name: "Test".to_string(),
            version: 4,
            snapshot: None,
            params: Parameters {
                waypoints: vec![
                    // Unfixed waypoint: a free initial guess the solver places
                    // itself, so it must not be perturbed (bounds are zero).
                    Waypoint::<Expr> {
                        x: Expr::new("1 m", 1.0),
                        y: Expr::new("1 m", 1.0),
                        heading: Expr::new("0 rad", 0.0),
                        dx: Expr::new("0 m", 0.0),
                        dy: Expr::new("0 m", 0.0),
                        dtheta: Expr::new("0 deg", 0.0),
                        intervals: 20,
                        split: false,
                        fix_translation: false,
                        fix_heading: false,
                        override_intervals: false,
                        is_initial_guess: false,
                    },
                    // Fixed waypoint: constrains the path, so it is optimizable
                    // within its bounds.
                    Waypoint::<Expr> {
                        x: Expr::new("2 m", 2.0),
                        y: Expr::new("2 m", 2.0),
                        heading: Expr::new("0 rad", 0.0),
                        dx: Expr::new("0.5 m", 0.5),
                        dy: Expr::new("0.5 m", 0.5),
                        dtheta: Expr::new("10 deg", 0.174_532_925_199_432_95),
                        intervals: 20,
                        split: false,
                        fix_translation: true,
                        fix_heading: true,
                        override_intervals: false,
                        is_initial_guess: false,
                    },
                ],
                constraints: Vec::new(),
                target_dt: Expr::new("0.05 s", 0.05),
            },
            trajectory: crate::spec::trajectory::Trajectory {
                config: None,
                sample_type: None,
                waypoints: Vec::new(),
                samples: Vec::new(),
                splits: Vec::new(),
            },
            events: Vec::new(),
        }
    }

    #[test]
    fn perturb_expr_preserves_unit() {
        let expr = Expr::new("1.5 m", 1.5);
        let out = perturb_expr(&expr, 0.1, "m");
        assert_eq!(out.val, 1.6);
        assert_eq!(out.exp, "1.6 m");

        let rad = Expr::new("3.14159 rad", std::f64::consts::PI);
        let out = perturb_expr(&rad, 0.001, "rad");
        assert_eq!(out.exp, "3.14259 rad");
    }

    #[test]
    fn perturb_expr_falls_back_to_default_unit() {
        // Non-literal expressions fall back to the default unit.
        let expr = Expr::new("2 * x + 1 m", 3.0);
        let out = perturb_expr(&expr, 0.25, "m");
        assert_eq!(out.val, 3.25);
        assert_eq!(out.exp, "3.25 m");
    }

    #[test]
    fn optimizable_dofs_perturbs_fixed_waypoints_only() {
        let file = test_trajectory();
        // Waypoint 0 is unfixed, so the solver treats it as a free initial
        // guess and places it itself; perturbing it cannot change the path.
        assert!(optimizable_dofs(&file, 0).is_empty());
        // Waypoint 1 fixes the path, so it is optimizable within its bounds.
        assert_eq!(
            optimizable_dofs(&file, 1),
            vec![(Dof::Dx, 0.5), (Dof::Dy, 0.5), (Dof::Dtheta, 0.174_532_925_199_432_95)]
        );
    }

    #[test]
    fn optimizable_dofs_respects_fix_flags_and_zero_bounds() {
        let file = test_trajectory();
        // A fixed waypoint with zero bounds has no optimizable dofs.
        let mut zeroed = file.clone();
        let wpt = &mut zeroed.params.waypoints[1];
        wpt.dx = Expr::new("0 m", 0.0);
        wpt.dy = Expr::new("0 m", 0.0);
        wpt.dtheta = Expr::new("0 deg", 0.0);
        assert!(optimizable_dofs(&zeroed, 1).is_empty());
        // An unfixed waypoint is not optimizable regardless of its bounds.
        let mut fixed = file.clone();
        let wpt = &mut fixed.params.waypoints[0];
        wpt.fix_translation = true;
        wpt.fix_heading = true;
        wpt.dx = Expr::new("1 m", 1.0);
        assert_eq!(optimizable_dofs(&fixed, 0), vec![(Dof::Dx, 1.0)]);
    }

    #[test]
    fn candidate_perturbs_the_requested_dof() {
        let file = test_trajectory();
        let out = candidate(&file, 0, Dof::Dx, 0.5, 1.0);
        assert_eq!(out.params.waypoints[0].x.val, 1.5);
        assert_eq!(out.params.waypoints[0].y.val, 1.0);
        assert_eq!(out.params.waypoints[0].heading.val, 0.0);
    }
}
