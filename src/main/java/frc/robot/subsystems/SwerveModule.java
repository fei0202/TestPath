package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.FSLib2025.control.PID;
import frc.FSLib2025.math.Maths;
import frc.FSLib2025.swerve.OnboardModuleState;
import frc.FSLib2025.swerve.SwerveModuleConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    public int ModuleNumber;
    public SwerveModuleConstants ModuleConstants;

    private TalonFX steerMotor;
    private TalonFX driveMotor;

    private CANcoder steerCANcoder;

    private PID steerPID;

    private Rotation2d lastAngle;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.ModuleNumber = moduleNumber;
        this.ModuleConstants = moduleConstants;

        steerPID = new PID(
                SwerveConstants.STEER_MOTOR_KP,
                SwerveConstants.STEER_MOTOR_KI,
                SwerveConstants.STEER_MOTOR_KD);

        lastAngle = new Rotation2d();

        steerMotor = new TalonFX(moduleConstants.SteerMotorId, RobotConstants.CANBUS_NAME);
        TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

        driveMotor = new TalonFX(moduleConstants.DriveMotorId, RobotConstants.CANBUS_NAME);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_MOTOR_GEAR_RATIO;
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveMotorConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        steerCANcoder = new CANcoder(moduleConstants.CANcoderId, RobotConstants.CANBUS_NAME);
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        steerCANcoder.getConfigurator().apply(cancoderConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = OnboardModuleState.optimize(desiredState, getModuleState().angle);

        desiredState.angle = Math.abs(desiredState.speedMetersPerSecond) < 0.01 ? lastAngle : desiredState.angle;
        double error = getModuleState().angle.getDegrees() - desiredState.angle.getDegrees();
        error = Maths.constrainAngleDegrees(error);
        double steerOutput = steerPID.calculate(error);
        setSteerMotor(steerOutput);
        lastAngle = desiredState.angle;

        double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_MODULE_SPEED;
        percentOutput = Maths.clamp(percentOutput, -1, 1);
        setDriveMotor(percentOutput);
    }

    private void setSteerMotor(double speed) {
        steerMotor.set(speed);
    }

    private void setDriveMotor(double speed) {
        driveMotor.set(speed);
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_PERIMETER,
                getSteerAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_PERIMETER,
                getSteerAngle());
    }

}