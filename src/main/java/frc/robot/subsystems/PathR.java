package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandFactory.PathplannerFactory;

public class PathR {
    private final CommandXboxController joystick;
    private final XboxController joystick2;
    private Command currentPathCommand;

    public PathR(CommandXboxController joystick, XboxController joystick2) {
        this.joystick = joystick;
        this.joystick2 = joystick2;
    }

    public void executeRPath(int index) {
        String pathName = "CSR_R" + (char) ('A' + index - 1);
        System.out.println("[PathR] Requested execution of path: " + pathName);
        runPath(pathName);
    }

    private void runPath(String pathName) {
        System.out.println("[PathR] Preparing to execute path: " + pathName);

        try {
            currentPathCommand = PathplannerFactory.driveThenFollowPath(pathName);

            if (currentPathCommand != null && !(currentPathCommand instanceof InstantCommand)) {
                System.out.println("[PathR] Command created successfully");
                CommandScheduler.getInstance().schedule(currentPathCommand);
                System.out.println("[PathR] Command scheduled successfully");
            } else {
                System.err.println("[PathR] Error: Failed to create or load path command for " + pathName);
            }
        } catch (FileVersionException | IOException | ParseException e) {
            System.err.println("[PathR] Failed to load PathPlanner path: " + pathName);
            e.printStackTrace();
        }
    }

    public void stopPath() {
        System.out.println("[PathR] Stopping current path execution.");
        if (currentPathCommand != null) {
            currentPathCommand.cancel();
            System.out.println("[PathR] Path execution cancelled.");
            currentPathCommand = null;
        } else {
            System.out.println("[PathR] No active path command to cancel.");
        }
    }
}
