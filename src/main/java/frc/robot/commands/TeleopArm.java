package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Arm;

public class TeleopArm extends Command {
    private final Arm arm;
    private final XboxController joystick1;

    private Rotation2d targetAngle;
    private double targetOffset;
    private double lastSpeed = 0;

    public TeleopArm(Arm arm, XboxController joystick1) {
        this.arm = arm;
        this.joystick1 = joystick1;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        targetOffset = CombinedControlConstants.ARM_DEFAULT_OFFSET;
        targetAngle = Rotation2d.fromRadians(arm.getAngle().getRadians());
    }

    @Override
    public void execute() {
        double currentAngle = arm.getAngle().getRadians();
        double joystickYmove = joystick1.getRightY();

        if (joystick1.getBButtonPressed()) {
            targetOffset = CombinedControlConstants.ARM_DEFAULT_OFFSET;
        } else if (joystick1.getXButtonPressed()) {
            targetOffset = CombinedControlConstants.ARM_CORAL_STATION_OFFSET;
        }
        targetAngle = Rotation2d.fromRadians(targetOffset);

        double pidOutput = arm.getPIDController().calculate(currentAngle, targetAngle.getRadians());
        double ffOutput = arm.getFeedforward().calculate(targetAngle.getRadians(), 0);
        double totalOutput = pidOutput + ffOutput;

        if (Math.abs(joystickYmove) > 0.2) {
            totalOutput = joystickYmove * CombinedControlConstants.ARM_SPEED;
        } else if (Math.abs(joystickYmove) < 0.2) {
            totalOutput = joystickYmove * CombinedControlConstants.ARM_BACKSPEED;
        }

        // !limit
        // double minAngle = CombinedControlConstants.ARM_MIN_ANGLE.getRadians();
        // double maxAngle = CombinedControlConstants.ARM_MAX_ANGLE.getRadians();
        // targetAngle = Rotation2d.fromRadians(MathUtil.clamp(targetAngle.getRadians(),
        // minAngle, maxAngle));

        totalOutput = MathUtil.clamp(totalOutput,
                lastSpeed - CombinedControlConstants.ARM_MAX_ARGULAR_ACCELERATION,
                lastSpeed + CombinedControlConstants.ARM_MAX_ARGULAR_ACCELERATION);
        lastSpeed = totalOutput;

        arm.setArmSpeed(totalOutput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmSpeed(0);
    }
}