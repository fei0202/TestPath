package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.CombinedControl;
import frc.robot.commands.QuickSetElevator;
import frc.robot.commands.TeleopClimb;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);

    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();

    private final TeleopClimb teleopClimb = new TeleopClimb(climb, joystick.getHID());
    private final QuickSetElevator quickSetElevator = new QuickSetElevator(elevator, joystick.getHID());
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final TeleopElevator teleopElevator = new TeleopElevator(elevator, joystick1.getHID(), null);
    // private final TeleopArm teleopArm = new TeleopArm(arm, joystick1.getHID());
    private final CombinedControl combinedControl1 = new CombinedControl(intake, arm, joystick1.getHID());
    private final TestSwerve testSwerve = new TestSwerve(swerve);

    public RobotContainer() {
        swerve.setDefaultCommand(teleopSwerve);
        elevator.setDefaultCommand(teleopElevator);
        // elevator.setDefaultCommand(quickSetElevator);
        // climb.setDefaultCommand(teleopClimb);
        intake.setDefaultCommand(combinedControl1);
        arm.setDefaultCommand(combinedControl1);
        // arm.setDefaultCommand(teleopArm);
        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {
        swerve.setDefaultCommand(testSwerve);
        return null;
    }
}