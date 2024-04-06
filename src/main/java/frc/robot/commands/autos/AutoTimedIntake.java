package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class AutoTimedIntake extends Command {
    private double duration, power;
    private Intake intake;

    public AutoTimedIntake(Intake intake, double duration, double power) {
        this.duration = duration;
        this.power = power;
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize() {
        double currentTime = System.currentTimeMillis();
        this.duration = currentTime + this.duration;
    }

    public void execute() {
        intake.SetIntake(power);
    }

    public boolean isFinished() {
        return System.currentTimeMillis() >= duration;
    }
    
    public void end() {
        intake.IntakeStop();
    }
}
