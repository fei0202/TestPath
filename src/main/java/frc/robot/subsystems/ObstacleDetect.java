package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class ObstacleDetect {
    private final AnalogInput ultrasonicSensor = new AnalogInput(0);

    public boolean isObstacleDetected() {
        double distance = ultrasonicSensor.getVoltage() * 100;
        return distance < 50;
    }
}