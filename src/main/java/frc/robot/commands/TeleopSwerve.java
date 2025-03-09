package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {

    private final Swerve swerve;
    private final CommandXboxController controller;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rotSpeed = 0.0;

    public TeleopSwerve(Swerve swerve, CommandXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (controller.getHID().getRawButtonPressed(8)) {
            swerve.setGyroYaw(0);
            swerve.setOdometryPosition(new Pose2d());
        }

        double joystickMagnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());

        double reduction = 0.3 + 0.7 * Math.pow(joystickMagnitude, 3);

        xSpeed = xLimiter.calculate(-controller.getLeftY()) * reduction * 0.8;
        ySpeed = yLimiter.calculate(-controller.getLeftX()) * reduction * 0.8;

        double rightX = controller.getRightX();
        if (Math.abs(rightX) > 0.2) {
            rotSpeed = rotLimiter.calculate(-rightX) * 0.6;
            rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);
        } else {
            rotSpeed = 0;
        }

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
