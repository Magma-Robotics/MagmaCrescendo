package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SetIndexer extends Command {
    private final Indexer indexer;
    private double power;
    
    public SetIndexer(Indexer indexer, double power) {
        this.power = power;
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.SetIndexer(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
