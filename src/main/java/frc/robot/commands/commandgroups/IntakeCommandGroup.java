package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(Intake intake, Indexer indexer, BeamBreakHandler beamBreakHandler) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new IntakeNoteCommandGroup(intake, indexer, beamBreakHandler));
    }
}