package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AutoTimedShooter extends Command {
    private double duration, power;
    private Shooter shooter;

    public AutoTimedShooter(Shooter shooter, double duration, double power) {
        this.duration = duration;
        this.power = power;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void initialize() {
        double currentTime = System.currentTimeMillis();
        this.duration = currentTime + this.duration;
    }

    public void execute() {
        shooter.SetShooter(power);
    }

    public boolean isFinished() {
        return System.currentTimeMillis() >= duration;
    }
    
    public void end() {
        shooter.ShooterStop();
    }
}
