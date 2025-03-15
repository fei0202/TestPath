package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.PathController;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);
    public CommandXboxController joystick2 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT2);

    private final Swerve swerve = new Swerve();
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final PathController pathController;

    public RobotContainer() {
        pathController = new PathController(joystick, joystick2);
        swerve.setDefaultCommand(teleopSwerve);
    }
}
