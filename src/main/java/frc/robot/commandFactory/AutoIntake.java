package frc.robot.commandFactory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CombinedControlConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class AutoIntake extends SequentialCommandGroup {
        private final Elevator elevator;
        private final Arm arm;
        private final Intake intake;

        public AutoIntake(Elevator elevator, Arm arm, Intake intake) {
                this.elevator = elevator;
                this.arm = arm;
                this.intake = intake;
                addRequirements(elevator, arm, intake);

                addCommands(
                                // new InstantCommand(() -> swerve.setOdometryPosition(new Pose2d()), swerve),
                                new RunCommand(() -> elevator
                                                .setDesiredHeight(ElevatorConstants.ELEVATOR_CORAL_STATION_HEIGHT),
                                                elevator).withTimeout(2),
                                new RunCommand(() -> intake.setIntakeSpeed(CombinedControlConstants.INTAKING_SPEED),
                                                intake)
                                                .withTimeout(4),
                                new RunCommand(() -> intake.setIntakeSpeed(0),
                                                intake)
                                                .withTimeout(1),

                                new InstantCommand(elevator::stop, elevator),
                                new InstantCommand(arm::stopArm, arm),
                                new InstantCommand(intake::stopIntake, intake));
        }
}
