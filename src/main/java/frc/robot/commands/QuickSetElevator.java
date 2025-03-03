package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class QuickSetElevator extends Command {
    private final Elevator elevator;
    private final PIDController leftController;
    private final PIDController rightController;
    private final XboxController joystick;
    private double targetPosition;
    private double lastSpeedL = 0;
    private double lastSpeedR = 0;

    public QuickSetElevator(Elevator elevator, XboxController joystick) {
        this.elevator = elevator;
        this.joystick = joystick;
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

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        targetPosition = elevator.getLeftPosition();
        leftController.setSetpoint(targetPosition);
        rightController.setSetpoint(targetPosition);
    }

    @Override
    public void execute() {
        double leftPosition = elevator.getLeftPosition();
        double rightPosition = elevator.getRightPosition();

        double newTarget = targetPosition;

        if (joystick.getRawButton(5)) {
            newTarget = 0;
        } else if (joystick.getXButtonPressed()) {
            newTarget = ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT;
        } else if (joystick.getYButtonPressed()) {
            newTarget = ElevatorConstants.ELEVATOR_L2_HEIGHT;
        } else if (joystick.getBButtonPressed()) {
            newTarget = ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT;
        } else if (joystick.getAButtonPressed()) {
            newTarget = ElevatorConstants.ELEVATOR_L4_HEIGHT;
        }

        if (newTarget != targetPosition) {
            targetPosition = newTarget;
            leftController.setSetpoint(targetPosition);
            rightController.setSetpoint(targetPosition);
        }

        double rawLeftOutput = leftController.calculate(leftPosition);
        double rawRightOutput = rightController.calculate(rightPosition);

        double leftOutput = MathUtil.clamp(
                rawLeftOutput, lastSpeedL - ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
                lastSpeedL + ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        double rightOutput = MathUtil.clamp(
                rawRightOutput, lastSpeedR - ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
                lastSpeedR + ElevatorConstants.ELEVATOR_MAX_ACCELERATION);

        lastSpeedL = leftOutput;
        lastSpeedR = rightOutput;

        // Set elevator speeds to the calculated outputs
        elevator.setElevatorSpeed(leftOutput, rightOutput);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPosition - elevator.getLeftPosition()) < ElevatorConstants.ELEVATOR_TOLERANCE &&
                Math.abs(targetPosition - elevator.getRightPosition()) < ElevatorConstants.ELEVATOR_TOLERANCE;
    }
}
