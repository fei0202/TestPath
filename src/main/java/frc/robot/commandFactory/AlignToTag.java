// package frc.robot.commandFactory;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;

// public class AlignToTag extends Command {
// private final Swerve swerve;
// private final Vision vision;

// public AlignToTag(Swerve swerve, Vision vision) {
// this.swerve = swerve;
// this.vision = vision;
// addRequirements(swerve, vision);
// }

// @Override
// public void initialize() {
// System.out.println("Starting AlignToTag");
// }

// @Override
// public void execute() {
// if (vision.hasTarget()) {
// double xOffset = vision.getXOffset();

// if (Math.abs(xOffset) > 1.0) { // 允許 1 度誤差
// double speed = xOffset * 0.05; // 設定轉向速度
// swerve.drive(0, 0, speed, true);
// } else {
// swerve.drive(0, 0, 0, true);
// }
// }
// }

// @Override
// public boolean isFinished() {
// return vision.hasTarget() && Math.abs(vision.getXOffset()) < 1.0;
// }

// @Override
// public void end(boolean interrupted) {
// swerve.drive(0, 0, 0, true);
// System.out.println("Finished AlignToTag");
// }
// }
