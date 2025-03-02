package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class QuickSetElevator extends Command {
    private final Elevator elevator;
    private final PIDController Lcontroller = new PIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD);
    private final PIDController Rcontroller = new PIDController(
            ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI,
            ElevatorConstants.ELEVATOR_KD);
    private final XboxController joystick;

    public QuickSetElevator(Elevator elevator, XboxController joystick) {
        this.elevator = elevator;
        this.joystick = joystick;
        addRequirements(elevator);
    }

    @Override
    public void execute() {

        double LeftPosition = elevator.getLeftPosition();
        double RightPosition = elevator.getRightPosition();

        SmartDashboard.putNumber("Left Position", LeftPosition);
        SmartDashboard.putNumber("Right Position", RightPosition);

        SmartDashboard.putData("Elevator Controller 1", Lcontroller);
        SmartDashboard.putData("Elevator Controller 2", Rcontroller);

        double NowPosition = SmartDashboard.getNumber("Elevator Setpoint", 0);
        if (joystick.getAButton()) {
            Lcontroller.setSetpoint(NowPosition);
            Rcontroller.setSetpoint(NowPosition);
        } else if (joystick.getBButton()) {
            Lcontroller.setSetpoint(0);
            Rcontroller.setSetpoint(0);
        }

        // 3X4Y2B1A
        if (joystick.getRawButton(3)) {
            Lcontroller.setSetpoint(ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT);
            Rcontroller.setSetpoint(ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT);
        } else if (joystick.getRawButton(4)) {
            Lcontroller.setSetpoint(ElevatorConstants.ELEVATOR_L2_HEIGHT);
            Rcontroller.setSetpoint(ElevatorConstants.ELEVATOR_L2_HEIGHT);
            // } else if(joystick.getRawButton(2)) {
            // Lcontroller.setSetpoint(ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT);
            // Rcontroller.setSetpoint(ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT);
            // }
            // else if(joystick.getRawButton(1)) {
            // Lcontroller.setSetpoint(ElevatorConstants.ELEVATOR_L4_HEIGHT);
            // Rcontroller.setSetpoint(ElevatorConstants.ELEVATOR_L4_HEIGHT);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
