package frc.robot.commandFactory;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class PathplannerFactory extends Command {

    public static Command driveToSetpointCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3.0,
                1.0,
                2 * Math.PI,
                4 * Math.PI);

        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.5);
    }

    public static Command driveThenFollowPath(String pathName) {
        PathConstraints constraints = new PathConstraints(
                3.0,
                1.0,
                2 * Math.PI,
                4 * Math.PI);

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch (FileVersionException | IOException | ParseException e) {
            System.err.println("can't load PathPlanner : " + pathName);
            e.printStackTrace();
            return new PrintCommand("can't load PathPlanner: " + pathName);
        }
    }
}
