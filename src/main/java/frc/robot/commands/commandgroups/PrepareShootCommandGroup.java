package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.commands.shooter.SpeakerRev;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class PrepareShootCommandGroup extends SequentialCommandGroup {
    public PrepareShootCommandGroup(Shooter shooter) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new SpeakerRev(shooter));
    }
}