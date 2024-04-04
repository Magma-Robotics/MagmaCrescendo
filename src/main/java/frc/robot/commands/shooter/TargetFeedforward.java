package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TargetFeedforward extends Command {
    private double velocity;
    private Shooter shooter;

    public TargetFeedforward(Shooter shooter, double velocity) {
        this.velocity = velocity;
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    public void execute() {
        shooter.TargetFeedforward(velocity);
    }

    public boolean isFinished() {
        return false;
    }
}