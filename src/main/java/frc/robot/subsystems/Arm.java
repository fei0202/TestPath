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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CombinedControlConstants;

public class Arm extends SubsystemBase {
    private final SparkMax ArmMotor;
    private final RelativeEncoder ArmEncoder;
    private final CANcoder ArmCancoder;
    private final PIDController AController;
    private final ArmFeedforward Afeedforward;
    private static final double ARM_CANCODER_OFFSET = CombinedControlConstants.ARM_DEFAULT_OFFSET;
    private double lastSpeed = 0;

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
        AController.setIntegratorRange(-0.2, 0.2);

        Afeedforward = new ArmFeedforward(
                CombinedControlConstants.ARM_KS,
                CombinedControlConstants.ARM_KG,
                CombinedControlConstants.ARM_KV,
                CombinedControlConstants.ARM_KA);

        setDesiredAngle(ARM_CANCODER_OFFSET);
    }

    public Rotation2d getAngle() {
        ArmCancoder.getAbsolutePosition().refresh();
        return Rotation2d.fromRadians(
                Math.toRadians(ArmCancoder.getAbsolutePosition().getValueAsDouble()) - ARM_CANCODER_OFFSET);
    }

    public void setDesiredAngle(double targetAngleRadians) {
        double currentAngle = getAngle().getRadians();
        double pidOutput = AController.calculate(currentAngle, targetAngleRadians);
        double ffOutput = Afeedforward.calculate(targetAngleRadians, 0);
        double rawOutput = pidOutput + ffOutput;

        double clampedOutput = MathUtil.clamp(rawOutput,
                lastSpeed - CombinedControlConstants.ARM_MAX_ARGULAR_ACCELERATION,
                lastSpeed + CombinedControlConstants.ARM_MAX_ARGULAR_ACCELERATION);
        lastSpeed = clampedOutput;
        setArmSpeed(clampedOutput);
    }

    public void setArmSpeed(double ASpeed) {
        ArmMotor.set(MathUtil.clamp(ASpeed, -CombinedControlConstants.ARM_SPEED, CombinedControlConstants.ARM_SPEED));
    }

    public PIDController getPIDController() {
        return AController;
    }

    public ArmFeedforward getFeedforward() {
        return Afeedforward;
    }

    public void stopArm() {
        ArmMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle (radians)", getAngle().getRadians());
        SmartDashboard.putNumber("Arm Cancoder Raw", ArmCancoder.getAbsolutePosition().getValueAsDouble());
    }
}
