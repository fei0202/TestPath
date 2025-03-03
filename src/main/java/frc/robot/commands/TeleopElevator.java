package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
    private final Elevator elevator;
    private final XboxController joystick1;

    public TeleopElevator(Elevator elevator, XboxController joystick1,
            edu.wpi.first.math.controller.ElevatorFeedforward feedforward) {
        this.elevator = elevator;
        this.joystick1 = joystick1;
        addRequirements(elevator);
    }

    @Override
    public void execute() {

        // Y/A elevator up down
        // if (joystick.getRawButton(4)) {
        // elevator.setElevatorSpeed(ElevatorConstants.ELEVATOR_SPEED,-ElevatorConstants.ELEVATOR_SPEED);
        // } else if (joystick.getRawButton(1)) {
        // elevator.setElevatorSpeed(-ElevatorConstants.ELEVATOR_SPEED,
        // ElevatorConstants.ELEVATOR_SPEED);
        // } else {
        // elevator.setElevatorSpeed(0,0);
        // }

        if (joystick1.getLeftY() > 0.5) {
            elevator.setElevatorSpeed(ElevatorConstants.ELEVATOR_SPEED, ElevatorConstants.ELEVATOR_SPEED);
        } else if (joystick1.getLeftY() < -0.5) {
            elevator.setElevatorSpeed(-ElevatorConstants.ELEVATOR_SPEED, -ElevatorConstants.ELEVATOR_SPEED);
        } else {
            elevator.setElevatorSpeed(0, 0);
        }

        double LeftPosition = elevator.getLeftPosition();
        double RightPosition = elevator.getRightPosition();

        SmartDashboard.putNumber("Left Elevator Position", LeftPosition);
        SmartDashboard.putNumber("Right Elevator Position", RightPosition);

    }
}