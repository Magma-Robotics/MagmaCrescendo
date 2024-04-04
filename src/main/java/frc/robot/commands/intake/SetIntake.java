package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntake extends Command {
    private final Intake intake;
    private double power;
    
    public SetIntake(Intake intake, double power) {
        this.power = power;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.SetIntake(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
