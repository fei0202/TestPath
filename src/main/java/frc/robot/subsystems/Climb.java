package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.Constants.RobotConstants;

public class Climb extends SubsystemBase {
    private final TalonFX ClimbMotor = new TalonFX(CombinedControlConstants.CLIMB_MOTOR_ID, RobotConstants.CANBUS_NAME);
    private final static TalonFXConfiguration ClimbMotorConfig = new TalonFXConfiguration();

    public Climb() {
        ClimbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ClimbMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        ClimbMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        ClimbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        ClimbMotor.getConfigurator().apply(ClimbMotorConfig);
    }

    public void setClimbSpeed(double speed) {
        ClimbMotor.set(speed);
    }

    public void stopClimb() {
        ClimbMotor.set(0);
    }

}
