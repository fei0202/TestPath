package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightTable;

    public Vision() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTagYaw() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTagDistance() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }
}
