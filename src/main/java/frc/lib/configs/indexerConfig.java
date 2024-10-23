package frc.lib.configs;

public class indexerConfig {
    public final int shooterIntakeMotor;
    public final double shooterIntakeSpeed;
    public final double indexerFeedSpeed;

    public indexerConfig(int intakeMotor, double intakeSpeed, double indexerSpeed) {
        this.shooterIntakeMotor=intakeMotor;
        this.shooterIntakeSpeed = intakeSpeed;
        this.indexerFeedSpeed = indexerSpeed;

    }
}
