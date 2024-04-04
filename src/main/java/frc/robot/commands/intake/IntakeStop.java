package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeStop extends Command {
    private final Intake intake;
    
    public IntakeStop(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.IntakeStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
