package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.PathController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public XboxController joystick2 = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);

    private final Swerve swerve = new Swerve();
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final PathController pathController = new PathController(joystick, joystick2, swerve);

    public RobotContainer() {
        swerve.setDefaultCommand(teleopSwerve);
    }

    public Swerve getSwerve() {
        return swerve;
    }

    public PathController getPathController() {
        return pathController;
    }
}