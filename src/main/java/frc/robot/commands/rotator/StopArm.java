package frc.robot.commands.rotator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rotator;

public class StopArm extends Command {
    private Rotator rotator;

    public StopArm(Rotator rotator) {
        this.rotator = rotator;
        addRequirements(rotator);
    }

    public void execute() {
        rotator.StopArm();
    }

    public boolean isFinished() {
        return false;
    }
}
