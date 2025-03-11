package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commandFactory.AutoB3;
import frc.robot.commandFactory.AutoIntake;
import frc.robot.commandFactory.AutoL1;
import frc.robot.commandFactory.PathplannerFactory;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopClimb;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObstacleDetect;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);
    public CommandXboxController joystick2 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT2);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<String> pathChooser = new SendableChooser<>();

    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();
    private final ObstacleDetect detector = new ObstacleDetect();

    private final TeleopClimb teleopClimb = new TeleopClimb(climb, joystick.getHID());
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final TeleopElevator teleopElevator = new TeleopElevator(elevator, joystick1.getHID(), joystick.getHID());
    private final TestSwerve testSwerve = new TestSwerve(swerve);
    private final TeleopArm teleopArm = new TeleopArm(arm, joystick1.getHID(), joystick.getHID());
    private final TeleopIntake teleopIntake = new TeleopIntake(intake, joystick1.getHID());

    public RobotContainer() {
        swerve.setDefaultCommand(teleopSwerve);
        elevator.setDefaultCommand(teleopElevator);
        arm.setDefaultCommand(teleopArm);
        intake.setDefaultCommand(teleopIntake);
        climb.setDefaultCommand(teleopClimb);

        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("AutoL1", new AutoL1(elevator, arm, swerve, intake));
        autoChooser.addOption("Basic Auto", new SequentialCommandGroup(
                new RunCommand(() -> swerve.drive(new Translation2d(0.3, 0), 0, false), swerve).withTimeout(2),
                new RunCommand(() -> swerve.drive(new Translation2d(0, 0), 0, false), swerve).withTimeout(1)));
        autoChooser.addOption("Path Planner Auto", new SequentialCommandGroup(
                PathplannerFactory.driveThenFollowPath(pathChooser.getSelected())));

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        swerve.setGyroYaw(0);
        swerve.setOdometryPosition(new Pose2d());

        // return new PathPlannerAuto("L1");
        // return new PathPlannerAuto("L1_CSL_B3");

        return autoChooser.getSelected();
    }

    public void generateNamedCommands() {
        NamedCommands.registerCommand("ForwardL1", new AutoL1(elevator, arm, swerve, intake));
        NamedCommands.registerCommand("INTAKE", new AutoIntake(elevator, arm, intake));
        NamedCommands.registerCommand("B3", new AutoB3(swerve, intake, elevator, arm));
    }
}
