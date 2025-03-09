package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CombinedControlConstants;

public class Arm extends SubsystemBase {
    private final SparkMax ArmMotor;
    private final RelativeEncoder ArmEncoder;
    private final CANcoder cancoder;
    private final PIDController ArmController;
    private static final double DEADZONE = 0.05;

    public Arm() {
        ArmController = new PIDController(CombinedControlConstants.ARM_KP, CombinedControlConstants.ARM_KI,
                CombinedControlConstants.ARM_KD);
        ArmMotor = new SparkMax(CombinedControlConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        ArmEncoder = ArmMotor.getEncoder();
        cancoder = new CANcoder(CombinedControlConstants.ARM_CANCODER_ID);

        SparkMaxConfig ArmConfig = new SparkMaxConfig();
        ArmConfig.idleMode(IdleMode.kBrake);

        resetArmPosition();
    }

    public double getArmAngle() {
        if (cancoder.getAbsolutePosition().getValueAsDouble() < 0) {
            return (cancoder.getAbsolutePosition().getValueAsDouble() + 1);// -0.46+1=0.54
        } else {
            return cancoder.getAbsolutePosition().getValueAsDouble();
        }
    }

    public void resetArmPosition() {
        ArmEncoder.setPosition(0);
    }

    public void setArmSpeed(double ASpeed) {
        ArmMotor.set(ASpeed);
    }

    public void setArmAngle(double setpoint) {
        double pidOutput = ArmController.calculate(getArmAngle(), setpoint);
        SmartDashboard.putNumber("arm controller output", pidOutput);
        setArmSpeed(pidOutput);
    }

    public void stopArm() {
        ArmMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm CANCoder Position", cancoder.getAbsolutePosition().getValueAsDouble());
    }
}
