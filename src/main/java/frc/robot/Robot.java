package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.FSLib2025.utils.LocalADStarAK;
import frc.robot.Constants.RobotConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        super(RobotConstants.PERIODIC_INTERVAL);
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        // new PathPlannerAuto("L1").schedule();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void teleopPeriodic() {
    }
}
