package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CombinedControlConstants;

public class Intake extends SubsystemBase {
    private final SparkMax IntakeMotor = new SparkMax(CombinedControlConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final Timer timer = new Timer();
    private static final double CURRENT_LIMIT = 40;

    public double getIntakeCurrent() {
        return IntakeMotor.getOutputCurrent();
    }

    public void setIntakeSpeed(double intakeSpeed) {
        double current = getIntakeCurrent();
        SmartDashboard.putNumber("Intake Motor Current", current);

        if (current > CURRENT_LIMIT) {
            stopIntake();
            SmartDashboard.putBoolean("Intake Overcurrent", true);
        } else {
            IntakeMotor.set(intakeSpeed);
            SmartDashboard.putBoolean("Intake Overcurrent", false);
        }
    }

    public void startIntake(double intakeSpeed) {
        IntakeMotor.set(intakeSpeed);
    }

    public void stopIntake() {
        IntakeMotor.stopMotor();
        SmartDashboard.putBoolean("Intake Stopped", true);
    }

}