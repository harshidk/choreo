#[cfg(test)]
mod scratch {
    use std::fs;

    use crate::{
        generation::{generate::generate},
        spec::{Expr, project::ProjectFile, trajectory::TrajectoryFile},
    };

    fn load() -> (ProjectFile, TrajectoryFile) {
        let project =
            ProjectFile::from_content(&fs::read_to_string("../test-jsons/project/0/swerve.chor").unwrap())
                .unwrap();
        let trajectory =
            TrajectoryFile::from_content(&fs::read_to_string("../test-jsons/trajectory/4/swerve.traj").unwrap())
                .unwrap();
        (project, trajectory)
    }

    #[test]
    fn perturb_fixed_vs_empty() {
        let (project, trajectory) = load();

        let base = generate(project.clone(), trajectory.clone(), 0).unwrap();
        let base_time = base.trajectory.samples.last().unwrap().t();
        println!("base: {base_time}");

        // Perturb waypoint 1 (empty waypoint: fixTranslation false, fixHeading false)
        let mut t1 = trajectory.clone();
        t1.params.waypoints[1].x = Expr::new("2.5 m", 2.5);
        t1.params.waypoints[1].y = Expr::new("2.5 m", 2.5);
        let g1 = generate(project.clone(), t1, 0).unwrap();
        let t1_time = g1.trajectory.samples.last().unwrap().t();
        println!("perturb empty wpt1 (2,2)->(2.5,2.5): {t1_time}");

        // Perturb waypoint 0 (pose waypoint: fixTranslation true, fixHeading true)
        let mut t2 = trajectory.clone();
        t2.params.waypoints[0].x = Expr::new("1.5 m", 1.5);
        t2.params.waypoints[0].y = Expr::new("1.5 m", 1.5);
        let g2 = generate(project.clone(), t2, 0).unwrap();
        let t2_time = g2.trajectory.samples.last().unwrap().t();
        println!("perturb pose wpt0 (1,1)->(1.5,1.5): {t2_time}");

        // Perturb waypoint 2 (pose waypoint: fixTranslation true, fixHeading true)
        let mut t3 = trajectory.clone();
        t3.params.waypoints[2].x = Expr::new("3.5 m", 3.5);
        t3.params.waypoints[2].y = Expr::new("3.5 m", 3.5);
        let g3 = generate(project.clone(), t3, 0).unwrap();
        let t3_time = g3.trajectory.samples.last().unwrap().t();
        println!("perturb pose wpt2 (3,3)->(3.5,3.5): {t3_time}");

        // Perturb waypoint 1 heading only (empty wpt, fixHeading false)
        let mut t4 = trajectory.clone();
        t4.params.waypoints[1].heading = Expr::new("2.0 rad", 2.0);
        let g4 = generate(project.clone(), t4, 0).unwrap();
        let t4_time = g4.trajectory.samples.last().unwrap().t();
        println!("perturb empty wpt1 heading -> 2.0 rad: {t4_time}");
    }
}
