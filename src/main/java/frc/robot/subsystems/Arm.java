package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CombinedControlConstants;

public class Arm extends SubsystemBase {
    private final SparkMax ArmMotor;
    private final RelativeEncoder ArmEncoder;
    private final CANcoder ArmCancoder;
    private final PIDController AController;
    private final ArmFeedforward Afeedforward;
    private static final double cancoderOffset = -0.005859;

    public Arm() {
        ArmMotor = new SparkMax(CombinedControlConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        ArmEncoder = ArmMotor.getEncoder();
        ArmCancoder = new CANcoder(CombinedControlConstants.ARM_CANCODER_ID);

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = cancoderConfig.MagnetSensor;

        magnetConfig.SensorDirection = CombinedControlConstants.ARM_CANCODER_REVERSED
                ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        ArmCancoder.getConfigurator().apply(cancoderConfig);

        SparkMaxConfig ArmConfig = new SparkMaxConfig();
        ArmConfig.idleMode(IdleMode.kBrake);
        ArmEncoder.setPosition(0);

        AController = new PIDController(
                CombinedControlConstants.ARM_KP,
                CombinedControlConstants.ARM_KI,
                CombinedControlConstants.ARM_KD);

        Afeedforward = new ArmFeedforward(
                CombinedControlConstants.ARM_KS, // 靜態摩擦力
                CombinedControlConstants.ARM_KG, // 重力
                CombinedControlConstants.ARM_KV, // 速度
                CombinedControlConstants.ARM_KA // 加速度
        );
    }

    public double getArmAngleRadians() {
        ArmCancoder.getAbsolutePosition().refresh();
        return Math.toRadians(ArmCancoder.getAbsolutePosition().getValueAsDouble() -
                cancoderOffset);
    }

    public void setDesiredAngle(double targetAngleRadians) {
        double currentAngle = getArmAngleRadians();
        double pidOutput = AController.calculate(currentAngle, targetAngleRadians);
        double ffOutput = Afeedforward.calculate(targetAngleRadians, 0);
        setArmVoltage(pidOutput + ffOutput);
    }

    public void setArmVoltage(double voltage) {
        ArmMotor.setVoltage(voltage);
    }

    public void stopArm() {
        ArmMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle (radians)", getArmAngleRadians());
        SmartDashboard.putNumber("Arm Motor Position", ArmEncoder.getPosition());
        SmartDashboard.putNumber("Arm Cancoder Raw",
                ArmCancoder.getAbsolutePosition().getValueAsDouble());
    }
}
