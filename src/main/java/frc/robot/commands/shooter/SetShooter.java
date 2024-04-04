package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    private final Shooter shooter;
    private double power;
    
    public  SetShooter(Shooter shooter, double power) {
        this.power = power;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.SetShooter(power);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
