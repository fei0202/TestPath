package frc.robot;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commandFactory.PathplannerFactory;
import frc.robot.commands.CombinedControl;
import frc.robot.commands.QuickSet;
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
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    public CommandXboxController joystick = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT0);
    public CommandXboxController joystick1 = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT1);

    private final Climb climb = new Climb();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Swerve swerve = new Swerve();

    private final PIDController leftController = new PIDController(Constants.ElevatorConstants.ELEVATOR_KP,
            Constants.ElevatorConstants.ELEVATOR_KI, Constants.ElevatorConstants.ELEVATOR_KD);
    private final PIDController rightController = new PIDController(Constants.ElevatorConstants.ELEVATOR_KP,
            Constants.ElevatorConstants.ELEVATOR_KI, Constants.ElevatorConstants.ELEVATOR_KD);
    private final PIDController AController = new PIDController(Constants.CombinedControlConstants.ARM_KP,
            Constants.CombinedControlConstants.ARM_KI, Constants.CombinedControlConstants.ARM_KD);

    private final TeleopClimb teleopClimb = new TeleopClimb(climb, joystick.getHID());
    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, joystick);
    private final TeleopElevator teleopElevator = new TeleopElevator(elevator, joystick1.getHID(), null);
    private final CombinedControl combinedControl = new CombinedControl(intake, arm, joystick1.getHID());
    private final QuickSet quickSet = new QuickSet(elevator, arm, joystick.getHID(),
            leftController,
            rightController,
            AController);
    private final TestSwerve testSwerve = new TestSwerve(swerve);
    private final TeleopArm teleopArm = new TeleopArm(arm, joystick1.getHID());
    private final TeleopIntake teleopIntake = new TeleopIntake(intake, joystick1.getHID());

    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private static final String PATH_FOLDER = "/home/lvuser/deploy/pathplanner/paths";

    public RobotContainer() {
        swerve.setDefaultCommand(teleopSwerve);
        elevator.setDefaultCommand(teleopElevator);
        // intake.setDefaultCommand(combinedControl);
        // arm.setDefaultCommand(combinedControl);
        // climb.setDefaultCommand(teleopClimb);
        arm.setDefaultCommand(teleopArm);
        intake.setDefaultCommand(teleopIntake);
        // elevator.setDefaultCommand(quickSet);
        // arm.setDefaultCommand(quickSet);

        pathChooser.setDefaultOption("None", "None");
        loadPathOptions();
        SmartDashboard.putData("Path Selection", pathChooser);

        configureButtonBindings();
    }

    private void loadPathOptions() {

        File pathDir = new File(PATH_FOLDER);
        File[] pathFiles = pathDir.listFiles((dir, name) -> name.endsWith(".path"));

        if (pathFiles != null && pathFiles.length > 0) {
            for (File pathFile : pathFiles) {
                String pathName = pathFile.getName().replace(".path", "");
                pathChooser.addOption(pathName, pathName);
            }
        }
    }

    private void configureButtonBindings() {
        Command autoCommand = getAutonomousCommand();
        if (autoCommand != null) {
            joystick1.back().onTrue(autoCommand);
        } else {
            System.out.println("ERROR: getAutonomousCommand() returned null!");
        }

    }

    public Command getAutonomousCommand() {
        // swerve.setDefaultCommand(testSwerve);
        String selectedPath = pathChooser.getSelected();

        if (selectedPath == null || selectedPath.equals("None")) {
            return new PrintCommand("No autonomous path selected");
        }

        Command pathCommand = PathplannerFactory.driveThenFollowPath(selectedPath);

        if (pathCommand == null) {
            return new PrintCommand("ERROR: PathplannerFactory returned null for path: " + selectedPath);
        }

        return pathCommand;
    }

}
