package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.FeedNote;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShuffleNoteCommandGroup extends SequentialCommandGroup {
    public ShuffleNoteCommandGroup(Indexer indexer, Shooter shooter) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new FeedNote(indexer).withTimeout(0.2), new SendBackNoteCommandGroup(indexer, shooter).withTimeout(0.6));
    }
}