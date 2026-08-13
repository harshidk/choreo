#[cfg(test)]
mod optimize {
    use std::fs;

    use tokio::sync::oneshot;

    use crate::{
        generation::{generate::generate, optimize::optimize},
        spec::{Expr, project::ProjectFile, trajectory::TrajectoryFile},
    };

    /// The example swerve project and trajectory (v4) from the test corpus,
    /// with a perturbation budget added to a waypoint that fixes the path.
    fn load_example() -> (ProjectFile, TrajectoryFile) {
        let project =
            ProjectFile::from_content(&fs::read_to_string("../test-jsons/project/0/swerve.chor").unwrap())
                .unwrap();
        let mut trajectory =
            TrajectoryFile::from_content(&fs::read_to_string("../test-jsons/trajectory/4/swerve.traj").unwrap())
                .unwrap();

        // Only fixed waypoints constrain the path (they become pose/translation
        // equality constraints in the solver); unfixed waypoints are free
        // initial guesses the solver places on its own. The middle waypoint in
        // the example file is unfixed, so make it fixed and move it off the
        // direct line between the endpoints. With a perturbation budget, the
        // optimizer can then pull the path back toward the straight line and
        // shorten it.
        let waypoint = &mut trajectory.params.waypoints[1];
        waypoint.x = Expr::new("2 m", 2.0);
        waypoint.y = Expr::new("3 m", 3.0);
        waypoint.heading = Expr::new("90 deg", std::f64::consts::FRAC_PI_2);
        waypoint.fix_translation = true;
        waypoint.fix_heading = true;
        waypoint.dx = Expr::new("0.5 m", 0.5);
        waypoint.dy = Expr::new("0.5 m", 0.5);
        waypoint.dtheta = Expr::new("10 deg", 10.0_f64.to_radians());

        (project, trajectory)
    }

    fn total_time(file: &TrajectoryFile) -> f64 {
        file.trajectory.samples.last().unwrap().t()
    }

    #[test]
    fn optimize_reduces_trajectory_time() {
        let (project, trajectory) = load_example();

        // Baseline: regenerate the trajectory without any perturbation.
        let baseline = generate(project.clone(), trajectory.clone(), 0).unwrap();
        let baseline_time = total_time(&baseline);

        // Optimize by perturbing waypoints within their bounds.
        let (_kill, cancel) = oneshot::channel();
        let optimized = optimize(project, trajectory, 0, cancel).unwrap();
        let optimized_time = total_time(&optimized);

        println!("Baseline time: {baseline_time:.4} s");
        println!("Optimized time: {optimized_time:.4} s");
        println!(
            "Improvement: {:.4} s ({:.2}%)",
            baseline_time - optimized_time,
            (baseline_time - optimized_time) / baseline_time * 100.0
        );

        assert!(
            optimized_time < baseline_time,
            "expected optimization to reduce total trajectory time \
             (optimized {optimized_time:.4} s vs baseline {baseline_time:.4} s)"
        );
    }
}
