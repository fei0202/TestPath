package frc.robot.commandFactory;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightAprilTag {
    public static void main(String[] args) {
        // 取得 Limelight NetworkTable
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // 讀取 AprilTag 相關數據
        NetworkTableEntry idEntry = limelightTable.getEntry("tid"); // AprilTag ID
        NetworkTableEntry xEntry = limelightTable.getEntry("tx"); // X 偏移
        NetworkTableEntry yEntry = limelightTable.getEntry("ty"); // Y 偏移
        NetworkTableEntry areaEntry = limelightTable.getEntry("ta"); // 面積 (接近程度)

        while (true) {
            double tagId = idEntry.getDouble(-1); // 預設值 -1 代表沒偵測到
            double xOffset = xEntry.getDouble(0);
            double yOffset = yEntry.getDouble(0);
            double area = areaEntry.getDouble(0);

            // 如果偵測到 AprilTag
            if (tagId != -1) {
                System.out.println("偵測到 AprilTag ID: " + tagId);
                System.out.println("X 偏移: " + xOffset);
                System.out.println("Y 偏移: " + yOffset);
                System.out.println("面積: " + area);

                // 依據 AprilTag ID 執行不同動作
                if (tagId == 1) {
                    System.out.println("執行動作 A");
                    // 呼叫相對應的機器人控制程式碼，例如移動至目標點
                } else if (tagId == 2) {
                    System.out.println("執行動作 B");
                    // 執行其他行為，例如對齊 AprilTag
                }
            } else {
                System.out.println("未偵測到 AprilTag");
            }

            try {
                Thread.sleep(100); // 稍微延遲，避免過度讀取
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
