// package frc.robot.commands;

// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.ObstacleDetect;
// import frc.FSLib2025.utils.DynamicPathPlanner;

// public class DynamicAvoidanceCommand extends Command {
// private final Swerve swerve;
// private final ObstacleDetect detector;
// private PathPlannerTrajectory trajectory;
// private boolean replanNeeded = false;

// public DynamicAvoidanceCommand(Swerve swerve, PathPlannerTrajectory
// trajectory, ObstacleDetect detector) {
// this.swerve = swerve;
// this.trajectory = trajectory;
// this.detector = detector;
// addRequirements(swerve);
// }

// @Override
// public void execute() {
// if (detector.isObstacleDetected()) {
// System.out.println("Obstacle detected! Replanning...");
// replanNeeded = true;
// }

// if (replanNeeded) {
// trajectory = DynamicPathPlanner.generateNewPath(swerve.getOdometryPosition(),
// trajectory.getEndState().poseMeters);
// replanNeeded = false;
// }

// swerve.followPath(trajectory); // Follow the new trajectory
// }

// // @Override
// // public boolean isFinished() {
// // return swerve.isPathCompleted();
// // }
// }
