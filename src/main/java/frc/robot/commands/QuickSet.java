package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class QuickSet extends Command {
    private final Arm arm;
    private final Elevator elevator;
    private final PIDController leftController;
    private final PIDController rightController;
    private final PIDController AController;
    private final XboxController joystick;

    private Rotation2d targetAngle;
    private double targetPosition;
    private double lastSpeedL = 0;
    private double lastSpeedR = 0;

    public QuickSet(
            Elevator elevator,
            Arm arm,
            XboxController joystick,
            PIDController leftController,
            PIDController rightController,
            PIDController AController) {

        this.elevator = elevator;
        this.joystick = joystick;
        this.arm = arm;

        this.AController = new PIDController(
                CombinedControlConstants.ARM_KP,
                CombinedControlConstants.ARM_KI,
                CombinedControlConstants.ARM_KD);
        this.leftController = new PIDController(
                ElevatorConstants.ELEVATOR_KP,
                ElevatorConstants.ELEVATOR_KI,
                ElevatorConstants.ELEVATOR_KD);
        this.rightController = new PIDController(
                ElevatorConstants.ELEVATOR_KP,
                ElevatorConstants.ELEVATOR_KI,
                ElevatorConstants.ELEVATOR_KD);

        leftController.setIntegratorRange(-0.5, 0.5);
        rightController.setIntegratorRange(-0.5, 0.5);

        addRequirements(elevator, arm);
    }

    @Override
    public void initialize() {
        targetPosition = elevator.getLeftPosition();
        leftController.setSetpoint(targetPosition);
        rightController.setSetpoint(targetPosition);
        targetAngle = arm.getAngle();
        AController.setSetpoint(targetAngle.getRadians());
    }

    @Override
    public void execute() {
        double leftPosition = elevator.getLeftPosition();
        double rightPosition = elevator.getRightPosition();
        double armAngle = arm.getAngle().getRadians();

        double newTarget = targetPosition;
        double newAngle = targetAngle.getRadians();

        if (joystick.getRawButton(5)) {
            newTarget = 0;
        } else if (joystick.getXButtonPressed()) {
            // newAngle = CombinedControlConstants.ARM_DEFAULT_OFFSET;
            newTarget = ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT;
        } else if (joystick.getYButtonPressed()) {
            // newAngle = CombinedControlConstants.ARM_DEFAULT_OFFSET;
            newTarget = ElevatorConstants.ELEVATOR_L2_HEIGHT;
        } else if (joystick.getBButtonPressed()) {
            // newAngle = CombinedControlConstants.ARM_DEFAULT_OFFSET;
            newTarget = ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT;
            // newAngle = CombinedControlConstants.ARM_CORAL_STATION_OFFSET;
        } else if (joystick.getAButtonPressed()) {
            // newAngle = CombinedControlConstants.ARM_DEFAULT_OFFSET;
            newTarget = ElevatorConstants.ELEVATOR_L4_HEIGHT;
            // newAngle = CombinedControlConstants.ARM_PUT_OFFSET;
        }

        if (newTarget != targetPosition) {
            targetPosition = newTarget;
            leftController.setSetpoint(targetPosition);
            rightController.setSetpoint(targetPosition);
        }
        if (newAngle != targetAngle.getRadians()) {
            targetAngle = new Rotation2d(newAngle);
            AController.setSetpoint(targetAngle.getRadians());
        }

        double rawLeftOutput = leftController.calculate(leftPosition);
        double rawRightOutput = rightController.calculate(rightPosition);
        double rawArmOutput = AController.calculate(armAngle);

        double leftOutput = MathUtil.clamp(
                rawLeftOutput, lastSpeedL - ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
                lastSpeedL + ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        double rightOutput = MathUtil.clamp(
                rawRightOutput, lastSpeedR - ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
                lastSpeedR + ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        double armOutput = MathUtil.clamp(
                rawArmOutput, -CombinedControlConstants.ARM_SPEED, CombinedControlConstants.ARM_SPEED);

        lastSpeedL = leftOutput;
        lastSpeedR = rightOutput;

        elevator.setElevatorSpeed(leftOutput, rightOutput);
        arm.setArmSpeed(armOutput);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPosition - elevator.getLeftPosition()) < ElevatorConstants.ELEVATOR_TOLERANCE &&
                Math.abs(targetPosition - elevator.getRightPosition()) < ElevatorConstants.ELEVATOR_TOLERANCE &&
                Math.abs(targetAngle.getRadians()
                        - arm.getAngle().getRadians()) < CombinedControlConstants.ARM_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorSpeed(0, 0);
        arm.setArmSpeed(0);
    }
}
