
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final ElevatorFeedforward feedforward;
    private final PIDController Lcontroller;
    private final PIDController Rcontroller;
    private final SparkMax LeftElevatorMotor;
    private final SparkMax RightElevatorMotor;
    private final TrapezoidProfile.Constraints LeftConstraints;
    private final TrapezoidProfile.Constraints RightConstraints;

    public Elevator() {
        LeftElevatorMotor = new SparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        RightElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig LeftConfig = new SparkMaxConfig();
        LeftConfig.inverted(true);
        LeftConfig.idleMode(IdleMode.kBrake);
        SparkMaxConfig RightConfig = new SparkMaxConfig();
        RightConfig.inverted(false);
        RightConfig.idleMode(IdleMode.kBrake);

        LeftConstraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.ELEVATOR_MAX_VELOCITY,
                ElevatorConstants.ELEVATOR_MAX_ACCELERATION);

        RightConstraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.ELEVATOR_MAX_VELOCITY,
                ElevatorConstants.ELEVATOR_MAX_ACCELERATION);

        Lcontroller = new PIDController(
                ElevatorConstants.ELEVATOR_KP,
                ElevatorConstants.ELEVATOR_KI,
                ElevatorConstants.ELEVATOR_KD);

        Rcontroller = new PIDController(
                ElevatorConstants.ELEVATOR_KP,
                ElevatorConstants.ELEVATOR_KI,
                ElevatorConstants.ELEVATOR_KD);

        feedforward = new ElevatorFeedforward(
                ElevatorConstants.ELEVATOR_KS,
                ElevatorConstants.ELEVATOR_KG,
                ElevatorConstants.ELEVATOR_KV,
                ElevatorConstants.ELEVATOR_KA);
    }

    public void setElevatorSpeed(double leftSpeed, double rightSpeed) {
        LeftElevatorMotor.set(leftSpeed);
        RightElevatorMotor.set(rightSpeed);
    }

    public void resetElevatorPosition() {
        LeftElevatorMotor.getEncoder().setPosition(0);
        RightElevatorMotor.getEncoder().setPosition(0);
    }

    public double getLeftPosition() {
        return LeftElevatorMotor.getEncoder().getPosition();
    }

    public double getRightPosition() {
        return RightElevatorMotor.getEncoder().getPosition();
    }

    public void setElevatorVoltage(double voltage) {
        double leftPosition = getLeftPosition();
        double rightPosition = getRightPosition();

        // limit
        if (voltage > 0 && (leftPosition >= ElevatorConstants.ELEVATOR_MAX_POSITION ||
                rightPosition >= ElevatorConstants.ELEVATOR_MAX_POSITION))
            voltage = 0;
        if (voltage < 0 && (leftPosition <= ElevatorConstants.ELEVATOR_MIN_POSITION ||
                rightPosition <= ElevatorConstants.ELEVATOR_MIN_POSITION))
            voltage = 0;

        LeftElevatorMotor.setVoltage(voltage);
        RightElevatorMotor.setVoltage(voltage);
    }

    public void setDesiredHeight(double height) {
        // target height, use PID controller and feedforward to control the elevator's
        // position
        Lcontroller.setSetpoint(height);
        Rcontroller.setSetpoint(height);

        double leftOutput = Lcontroller.calculate(getLeftPosition());
        double rightOutput = Rcontroller.calculate(getRightPosition());

        double leftFeedforward = feedforward.calculate(Lcontroller.getSetpoint());
        double rightFeedforward = feedforward.calculate(Rcontroller.getSetpoint());

        setElevatorVoltage(leftOutput + leftFeedforward);
        setElevatorVoltage(rightOutput + rightFeedforward);
    }
}
