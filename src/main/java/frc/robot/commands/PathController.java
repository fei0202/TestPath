package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandFactory.PathplannerFactory;
import frc.robot.subsystems.PathL;
import frc.robot.subsystems.PathR;
import frc.robot.subsystems.Swerve;

public class PathController {
    private final PathL pathL;
    private final PathR pathR;
    private final Swerve swerve;
    private boolean isRight = false;

    public PathController(CommandXboxController joystick, XboxController joystick2, Swerve swerve) {
        this.swerve = swerve;
        this.pathL = new PathL(joystick, joystick2);
        this.pathR = new PathR(joystick, joystick2);

        configureButtonBindings(joystick, joystick2);
    }

    private void configureButtonBindings(CommandXboxController joystick, XboxController joystick2) {
        joystick.a().onTrue(new InstantCommand(this::togglePathSide));
        joystick.back().onTrue(new InstantCommand(this::stopPath));

        new Trigger(() -> joystick2.getAButtonPressed()).onTrue(new InstantCommand(() -> executePathByIndex(1)));
        new Trigger(() -> joystick2.getStartButtonPressed()).onTrue(
                new InstantCommand(() -> executePathToPose()));
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
        swerve.followPath(index);
    }

    private void stopPath() {
        pathL.stopPath();
        pathR.stopPath();
        swerve.stop();
    }

    private void executePathToPose() {
        Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
        Command pathfindingCommand = PathplannerFactory.driveToSetpointCommand(targetPose);

        if (pathfindingCommand != null) {
            System.out.println("Executing path to Pose: " + targetPose);
            CommandScheduler.getInstance().schedule(pathfindingCommand);
        } else {
            System.err.println("Failed to create pathfinding command to " + targetPose);
        }
    }
}