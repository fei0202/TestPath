package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {

    private final Swerve swerve;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rotSpeed = 0.0;

    private double reduction = 1;

    private double rotationTarget = 0.5;
    private PIDController rotationController = new PIDController(0.01, 0, 0);

    private CommandXboxController controller;

    public TeleopSwerve(Swerve swerve, CommandXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {

        if (controller.getHID().getRawButtonPressed(8)) {
            swerve.setOdometryPosition(new Pose2d());
        }

        if (controller.getRightTriggerAxis() > 0.2) {
            reduction = 0.3;
        } else {
            reduction = 1;
        }

        // rotationController.setSetpoint(rotationTarget);

        xSpeed = xLimiter.calculate(-controller.getLeftY());
        ySpeed = yLimiter.calculate(-controller.getLeftX());
        if (controller.getRightX() > 0.2) {
            rotSpeed = rotLimiter.calculate(-controller.getRightX() * reduction);
            rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);
        } else if (controller.getRightX() < -0.2) {
            rotSpeed = rotLimiter.calculate(controller.getRightX() * reduction);
            rotSpeed = Math.copySign(-rotSpeed * rotSpeed, -rotSpeed);
        } else {
            rotSpeed = 0;
        }

        // square the input to inprove driving experience
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);

        swerve.drive(
                new Translation2d(xSpeed, ySpeed).times(SwerveConstants.MAX_MODULE_SPEED),
                rotSpeed * SwerveConstants.MAX_MODULE_ROTATIONAL_SPEED,
                true);

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false);
    }
}