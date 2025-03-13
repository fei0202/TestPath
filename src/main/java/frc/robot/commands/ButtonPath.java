package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve; // 假設這是驅動系統的 Subsystem
import frc.robot.subsystems.Path1; // 假設這是路徑1相關的Subsystem
import frc.robot.subsystems.Path2; // 假設這是路徑2相關的Subsystem

public class ButtonPath extends Command {
    private final Swerve swerve;
    private final XboxController joystick2;
    private boolean isPath1Active = true; // 設定一個布林值來選擇路徑
    private final Path1 path1;
    private final Path2 path2;

    public ButtonPath(Swerve driveTrain, Path1 path1, Path2 path2, XboxController joystick2) {
        this.swerve = driveTrain;
        this.path1 = path1;
        this.path2 = path2;
        this.joystick2 = joystick2;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 切換路徑的邏輯
        if (joystick2.getAButtonPressed()) {
            isPath1Active = true; // 切換到 path1
            SmartDashboard.putString("Active Path", "Path 1");
        }
        if (joystick2.getBButtonPressed()) {
            isPath1Active = false; // 切換到 path2
            SmartDashboard.putString("Active Path", "Path 2");
        }

        // 根據當前選擇的路徑來執行對應的操作
        if (isPath1Active) {
            // 執行 path1 相關命令
            path1.executePath();
        } else {
            // 執行 path2 相關命令
            path2.executePath();
        }

        // 顯示當前選擇的路徑
        SmartDashboard.putBoolean("Is Path1 Active", isPath1Active);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
