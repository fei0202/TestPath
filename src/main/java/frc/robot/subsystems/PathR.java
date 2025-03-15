package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commandFactory.PathplannerFactory;

public class PathR {
    private final XboxController joystick;
    private final XboxController joystick2;
    private Command currentPathCommand;

    public PathR(XboxController joystick, XboxController joystick2) {
        this.joystick = joystick;
        this.joystick2 = joystick2;
    }

    public void executeRPath(int index) {
        String pathName = "CSR-R" + (char) ('A' + index - 1);
        runPath(pathName);
    }

    private void runPath(String pathName) {
        System.out.println("Executing path: " + pathName);
        currentPathCommand = PathplannerFactory.driveThenFollowPath(pathName);

        if (currentPathCommand != null) {
            CommandScheduler.getInstance().schedule(currentPathCommand);
        }
    }

    public void stopPath() {
        System.out.println("Path execution paused.");
        if (currentPathCommand != null) {
            currentPathCommand.cancel();
            currentPathCommand = null;
        }
    }
}