package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CombinedControlConstants;

public class Intake extends SubsystemBase {
    private final SparkMax IntakeMotor = new SparkMax(CombinedControlConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final double currentLimit = 30.0;
    private final Timer timer = new Timer();
    private boolean isOverCurrent = false;
    public static final SparkMaxConfig INTAKER_MOTOR_CONFIGURATION = new SparkMaxConfig();
    
    static {
        INTAKER_MOTOR_CONFIGURATION.inverted(false);
    }

    public Intake() {
        SparkMaxConfig IntakeConfig = new SparkMaxConfig();
        IntakeConfig.idleMode(IdleMode.kBrake);
 
    }

    public void setIntakeSpeed(double intakeSpeed) {
        double current = IntakeMotor.getOutputCurrent(); 

        if (current > currentLimit) {
            if (!isOverCurrent) {
                timer.reset();
                timer.start();
                isOverCurrent = true;
            }

            if (timer.hasElapsed(3.0)) { 
                IntakeMotor.set(0); 
            }
        } else {
            isOverCurrent = false;
            timer.stop();
            IntakeMotor.set(intakeSpeed); 
        }
    }

    public void stopIntake() {
        IntakeMotor.stopMotor(); 
    }
}
