package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterStop extends Command {
    private final Shooter shooter;
    
    public ShooterStop(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.ShooterStop();;

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
