package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerStop extends Command {
    private final Indexer indexer;
    
    public IndexerStop(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.IndexerStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
