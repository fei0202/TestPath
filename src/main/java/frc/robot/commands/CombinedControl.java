package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class CombinedControl extends Command {

    private final Intake intake;
    private final Arm arm;
    private final XboxController joystick1;
    private Timer intakeTimer = new Timer();
    private boolean isIntakeActive = false;

    private Rotation2d targetAngle;

    public CombinedControl(Intake intake, Arm arm, XboxController joystick1) {
        this.intake = intake;
        this.arm = arm;
        this.joystick1 = joystick1;
        addRequirements(intake, arm);
    }

    // @Override
    // public void initialize() {
    // targetAngle = Rotation2d.fromRadians(arm.getArmAngleRadians());
    // arm.setDesiredAngle(targetAngle.getRadians());
    // }

    @Override
    public void execute() {
        // double currentAngle = arm.getArmAngleRadians();

        // if (joystick1.getBButton()) { // B (2)
        // targetAngle = CombinedControlConstants.ARM_DEFAULT_ANGLE;
        // } else if (joystick1.getXButton()) { // X (3)
        // targetAngle = CombinedControlConstants.ARM_CORAL_STATION_ANGLE;
        // }

        if (joystick1.getBButton()) { // B (2)
            arm.setArmSpeed(CombinedControlConstants.ARM_SPEED);
        } else if (joystick1.getXButton()) { // X (3)
            arm.setArmSpeed(CombinedControlConstants.ARM_BACKSPEED);
        } else {
            arm.setArmSpeed(0);
        }

        // double joystickInput = joystick1.getRightY();
        // if (Math.abs(joystickInput) > 0.2) {
        // targetAngle = Rotation2d.fromRadians(targetAngle.getRadians() +
        // Math.toRadians(2) * joystickInput);
        // }

        if (joystick1.getYButton()) {
            intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED);
            intakeTimer.reset();
            intakeTimer.start();
            isIntakeActive = true;
        } else if (joystick1.getAButton()) {
            intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED);
            intakeTimer.reset();
            intakeTimer.start();
            isIntakeActive = true;
        }

        if (isIntakeActive && intakeTimer.get() >= CombinedControlConstants.Intake_DURATION) {
            intake.stopIntake();
            isIntakeActive = false;
        }

        // @Override
        // if (targetAngle.getRadians() <
        // CombinedControlConstants.ARM_MIN_ANGLE.getRadians()) {
        // targetAngle = CombinedControlConstants.ARM_MIN_ANGLE;
        // } else if (targetAngle.getRadians() >
        // CombinedControlConstants.ARM_MAX_ANGLE.getRadians()) {
        // targetAngle = CombinedControlConstants.ARM_MAX_ANGLE;
        // }

        // arm.setDesiredAngle(targetAngle.getRadians());

        if (joystick1.getRightY() > 0.2) {
            arm.setArmSpeed(CombinedControlConstants.ARM_SPEED);
        } else if (joystick1.getRightY() < -0.2) {
            arm.setArmSpeed(CombinedControlConstants.ARM_BACKSPEED);
        } else {
            arm.setArmSpeed(0);
        }

        // if (joystick1.getRawButton(4)) { // Y (4) put
        // intake.setIntakeSpeed(CombinedControlConstants.PLACE_SPEED);
        // } else if (joystick1.getRawButton(1)) { // A (1) intake
        // intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED);
        // } else {
        // intake.stopIntake();
        // }

    }
}
