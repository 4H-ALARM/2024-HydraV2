package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.commands.indexer.IndexNote;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeNoteCommandGroup extends ParallelDeadlineGroup {
    public IntakeNoteCommandGroup(Intake intake, Indexer indexer, BeamBreakHandler beamBreakHandler) {
        // TODO: Add your deadline and sequential commands in the super() call, e.g.
        //           super(new MyDeadlineCmd(), new OpenClawCommand(), new MoveArmCommand());
        //       Add the deadline command first. i.e. the command that upon 
        //       ending will interrupt all other commands that are still running
        super(new IntakeNote(intake, beamBreakHandler), new IndexNote(indexer));
    }
}