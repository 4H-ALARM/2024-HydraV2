package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;


public class IndexNote extends Command {
    private final Indexer indexer;

    public IndexNote(Indexer indexer) {
        this.indexer = indexer;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.indexer.startIndexer(this.indexer.INDEXER_INTAKE_VELOCITY.get());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.indexer.stopIndexer();
    }
}
