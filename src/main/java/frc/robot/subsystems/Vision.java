package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final Swerve swerve;

    public Vision(Swerve swerve) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.swerve = swerve;
    }

    public void correctPositionBasedOnAprilTag() {
        Pose2d aprilTagPose = getAprilTagPose();
        Pose2d targetPose = new Pose2d(5, 5, new Rotation2d(0));

        double offsetX = targetPose.getX() - aprilTagPose.getX();
        double offsetY = targetPose.getY() - aprilTagPose.getY();

        if (Math.abs(offsetX) > 0.1 || Math.abs(offsetY) > 0.1) {
            swerve.drive(new Translation2d(offsetX, offsetY), 0, false);
            System.out.println("Correcting position...");
        } else {
            System.out.println("Position is aligned with target.");
        }
    }

    public boolean isAprilTagDetected() {
        double id = limelightTable.getEntry("tid").getDouble(-1);
        return id != -1;
    }

    public double getAprilTagID() {
        return limelightTable.getEntry("tid").getDouble(-1);
    }

    public double getAprilTagX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getAprilTagY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double getAprilTagArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public Pose2d getAprilTagPose() {
        double tagX = getAprilTagX();
        double tagY = getAprilTagY();
        double tagRotation = 0;

        return new Pose2d(tagX, tagY, new Rotation2d(tagRotation));
    }
}
