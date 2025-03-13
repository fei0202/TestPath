package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commandFactory.AlignToTag;
import frc.robot.commandFactory.AutoIntake;
import frc.robot.commandFactory.AutoL2;
import frc.robot.commandFactory.AutonomousRoutine;
import frc.robot.commandFactory.ForwardL1;
import frc.robot.commandFactory.Offset;
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
import frc.robot.subsystems.Vision;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);
    public CommandXboxController joystick2 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT2);

    private final SendableChooser<Command> autoChooser;

    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();
    private final ObstacleDetect obstacledetector = new ObstacleDetect();
    private final Vision vision = new Vision();

    private final TeleopClimb teleopClimb = new TeleopClimb(climb, joystick.getHID());
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final TeleopElevator teleopElevator = new TeleopElevator(elevator, joystick1.getHID(), joystick.getHID());
    private final TestSwerve testSwerve = new TestSwerve(swerve);
    private final TeleopArm teleopArm = new TeleopArm(arm, joystick1.getHID(), joystick.getHID());
    private final TeleopIntake teleopIntake = new TeleopIntake(intake, joystick1.getHID());

    public RobotContainer() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

        swerve.setDefaultCommand(teleopSwerve);
        elevator.setDefaultCommand(teleopElevator);
        arm.setDefaultCommand(teleopArm);
        intake.setDefaultCommand(teleopIntake);
        climb.setDefaultCommand(teleopClimb);

        generateNamedCommands();

        loadPathPlannerAutos();
        // autoChooser.addOption("Forward L1", new ForwardL1(elevator, arm, swerve,
        // intake));
        // autoChooser.addOption("Correct Position", correctPositionCommand());
        // autoChooser.addOption("L1", new PathPlannerAuto("L1"));
        // autoChooser.addOption("L1_CSL_B3", new PathPlannerAuto("L1_CSL_B3"));

        SmartDashboard.putString("Selected Auto Path", autoChooser.getSelected().getName());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureButtonBindings();
    }

    private void loadPathPlannerAutos() {
        File pathFolder = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");

        if (pathFolder.exists() && pathFolder.isDirectory()) {
            File[] files = pathFolder.listFiles((dir, name) -> name.endsWith(".auto"));
            if (files != null) {
                System.out.println("Found " + files.length + " auto files in pathplanner/autos:");
                for (File file : files) {
                    String pathName = file.getName().replace(".auto", "");
                    autoChooser.addOption(pathName, new PathPlannerAuto(pathName));
                    System.out.println("Added Auto Path: " + pathName);
                }
            } else {
                System.out.println("No .auto files found in pathplanner/autos directory!");
            }
        } else {
            System.out.println("PathPlanner autos directory not found: " + pathFolder.getAbsolutePath());
        }
    }

    public void configureButtonBindings() {
        joystick2.start().onTrue(
                new SequentialCommandGroup(
                        new AlignToTag(vision, swerve),
                        new Offset(swerve, 0.5, 0, 2)));
        // 按 A 鍵 → 機器人朝向 AprilTag 旋轉對準。
        // 按 B 鍵 → 根據 AprilTag ID 來決定是否啟動 Intake。
        // joystick1.start().onTrue(new AlignToTag(swerve, vision));
        // joystick1.start().onTrue(new AutoIntakeByTag(intake, elevator, arm, vision));

    }

    public Command getAutonomousCommand() {
        swerve.setGyroYaw(0);
        swerve.setOdometryPosition(new Pose2d());

        // new SequentialCommandGroup(
        // new AlignToTagCommand(visionSubsystem, swerveDriveSubsystem), // 先對齊
        // new OffsetCommand(swerveDriveSubsystem, 0.5, 0, 2) // 再往前移動 2 秒
        // );

        // return new AlignToTag(swerve)
        // .andThen(new AutoIntakeByTag(intake));
        return autoChooser.getSelected();

        // return new AutonomousRoutine(swerve);
    }

    public void generateNamedCommands() {

        NamedCommands.registerCommand("AprilTagDetection", new AutonomousRoutine(vision, swerve));
        NamedCommands.registerCommand("ForwardL1", new ForwardL1(elevator, arm, swerve, intake));
        NamedCommands.registerCommand("INTAKE", new AutoIntake(elevator, arm, intake));
        NamedCommands.registerCommand("L3", new AutoL2(swerve, intake, elevator, arm));
    }
}
