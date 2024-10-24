package frc.lib.configs;

public class indexerConfig {
    public final int shooterIntakeMotor;
    public final double shooterIntakeSpeed;
    public final double indexSpeed;
    public final double indexerFeedBackSpeed;
    public final PIDConfig pidConfig;


    public indexerConfig(int intakeMotor, double intakeSpeed, double indexerSpeed, double feedback, double p, double i, double d, double f) {
        this.shooterIntakeMotor=intakeMotor;
        this.shooterIntakeSpeed = intakeSpeed;
        this.indexSpeed = indexerSpeed;
        this.indexerFeedBackSpeed = feedback;
        this.pidConfig = new PIDConfig(p, i, d, f);

    }
}
