package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.FeedBackNote;
import frc.robot.commands.shooter.SendBackShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SendBackNoteCommandGroup extends ParallelCommandGroup {
    public SendBackNoteCommandGroup(Indexer indexer, Shooter shooter) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new SendBackShooter(shooter), new FeedBackNote(indexer));
    }
}