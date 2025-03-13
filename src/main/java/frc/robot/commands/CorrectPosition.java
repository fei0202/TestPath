package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class CorrectPosition extends Command {
    private final Vision vision;

    public CorrectPosition(Vision vision) {
        this.vision = vision;
        addRequirements(vision);
    }

    @Override
    public void initialize() {

        vision.correctPositionBasedOnAprilTag();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
