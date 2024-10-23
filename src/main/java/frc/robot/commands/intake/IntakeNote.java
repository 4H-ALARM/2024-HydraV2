package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.subsystems.Intake;


public class IntakeNote extends Command {
    private final Intake intake;
    private final BeamBreakHandler beamBreak;

    public IntakeNote(Intake intake, BeamBreakHandler beamBreakHandler) {
        this.intake = intake;
        this.beamBreak = beamBreakHandler;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.pickUpNote();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return beamBreak.isSeen();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
