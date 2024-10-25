package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class AutoShootCommandGroup extends SequentialCommandGroup {
    public AutoShootCommandGroup(Indexer indexer, Shooter shooter, BeamBreakHandler beamBreakHandler) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new AutoShoot(indexer, shooter, beamBreakHandler));
    }
}