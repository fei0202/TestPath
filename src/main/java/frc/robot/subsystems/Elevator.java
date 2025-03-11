package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax LeftElevatorMotor;
    private final SparkMax RightElevatorMotor;
    private final PIDController LeftController;
    // private final PIDController RightController;

    public Elevator() {
        LeftElevatorMotor = new SparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        RightElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        LeftController = new PIDController(ElevatorConstants.ELEVATOR_LKP, ElevatorConstants.ELEVATOR_KI,
                ElevatorConstants.ELEVATOR_KD);
        // RightController = new PIDController(ElevatorConstants.ELEVATOR_RKP,
        // ElevatorConstants.ELEVATOR_KI,
        // ElevatorConstants.ELEVATOR_KD);

        SparkMaxConfig LeftConfig = new SparkMaxConfig();
        LeftConfig.idleMode(IdleMode.kBrake);
        LeftConfig.inverted(false);
        LeftElevatorMotor.configure(LeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig RightConfig = new SparkMaxConfig();
        RightConfig.idleMode(IdleMode.kBrake);
        RightConfig.follow(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, true);
        RightElevatorMotor.configure(RightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        resetElevatorPosition();
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

    public void setDesiredHeight(double height) {

        // SmartDashboard.putNumber("right controller output",
        // RightController.calculate(height, getRightPosition()));
        LeftElevatorMotor.set(-LeftController.calculate(height, getLeftPosition()));
        // RightElevatorMotor.set(-RightController.calculate(height,
        // getRightPosition()));
    }

    public double getLeftSpeed() {
        return -LeftElevatorMotor.get();
    }

    public double getRightSpeed() {
        return RightElevatorMotor.get();
    }

    public void stop() {
        LeftElevatorMotor.set(0);
        RightElevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Elevator Position", getLeftPosition());
        SmartDashboard.putNumber("Right Elevator Position", getRightPosition());
        SmartDashboard.putNumber("left motor output",
                LeftElevatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("right motor output",
                RightElevatorMotor.getAppliedOutput());
    }
}
