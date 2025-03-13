package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObstacleDetect extends SubsystemBase {
    private final AnalogInput ultrasonicSensor = new AnalogInput(0);

    // 不再需要接受 DifferentialDrive 作為建構子的參數
    public ObstacleDetect() {
        // 這裡可以初始化其他元件，若需要的話
    }

    public boolean isObstacleDetected() {
        double distance = ultrasonicSensor.getVoltage() * 100; // 轉換為距離 (cm)
        return distance < 50; // 假設距離小於 50cm 代表有障礙物
    }

    public void avoidObstacle() {
        if (isObstacleDetected()) {
            // 假設檢測到障礙物時，進行停止或移動到一個安全位置
            // 假設你有控制機器人的方式，這裡進行控制邏輯
            // 這裡可以添加對機器人行為的控制代碼，例如停止、轉向等
        }
    }
}
