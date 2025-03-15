package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class PathController {
    private final PathL pathL;
    private final PathR pathR;
    private boolean isRight = false;

    public PathController(CommandXboxController joystick, CommandXboxController joystick2) {
        this.pathL = new PathL(joystick.getHID(), joystick2.getHID());
        this.pathR = new PathR(joystick.getHID(), joystick2.getHID());

        configureButtonBindings(joystick, joystick2);
    }

    private void configureButtonBindings(CommandXboxController joystick, CommandXboxController joystick2) {
        joystick.button(1).onTrue(new InstantCommand(this::togglePathSide));
        joystick.button(7).onTrue(new InstantCommand(this::stopPath));

        for (int i = 1; i <= 12; i++) {
            int pathIndex = i;
            joystick2.button(i).onTrue(new InstantCommand(() -> executePathByIndex(pathIndex)));
        }
    }

    private void togglePathSide() {
        isRight = !isRight;
        System.out.println("Path side switched: " + (isRight ? "Right" : "Left"));
    }

    private void executePathByIndex(int index) {
        if (isRight) {
            pathR.executeRPath(index);
        } else {
            pathL.executeLPath(index);
        }
    }

    private void stopPath() {
        pathL.stopPath();
        pathR.stopPath();
    }
}
