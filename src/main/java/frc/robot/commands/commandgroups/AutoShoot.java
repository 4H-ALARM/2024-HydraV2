package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;


public class AutoShoot extends Command {
    private final Indexer indexer;
    private final Shooter shooter;
    private final BeamBreakHandler beamBreakHandler;

    public AutoShoot(Indexer indexer, Shooter shooter, BeamBreakHandler beamBreakHandler) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.beamBreakHandler = beamBreakHandler;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexer, this.shooter);
    }

    @Override
    public void initialize() {
        shooter.startShooter(shooter.SHOOTER_BASE_VELOCITY.get());
    }

    @Override
    public void execute() {
        if (shooter.isRevved(shooter.SHOOTER_BASE_VELOCITY.get())) {
            indexer.startIndexer(indexer.INDEXER_FEED_VELOCITY.get());
        }
    }

    @Override
    public boolean isFinished() {
        return !beamBreakHandler.isSeen();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
