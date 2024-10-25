package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.configs.shooterConfig;
import frc.robot.subsystems.Shooter;


public class AmpRev extends Command {
    private final Shooter shooter;

    public AmpRev(Shooter shooter) {
        this.shooter = shooter;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.shooter.startShooter(this.shooter.SHOOTER_AMP_VELOCITY.get());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
    }
}
