package frc.lib.configs;

public class shooterConfig {
    public final int shooterTopMotor;
    public final int shooterBottomMotor;
    public final double targetpassvelocity;
    public final double targetampvelocity;
    public final double targetsendbackvelocity;
    public final double targetbaseshootvelocity;
    public final double tolerance;
    public final PIDConfig pidConfig;


    public shooterConfig(int topMotor, int bottomMotor, double passvelocity, double ampvelocity, double sendbackvelocity, double basevelocity, double tolerance, double kp, double ki, double kd, double kf) {
        this.shooterTopMotor = topMotor;
        this.shooterBottomMotor = bottomMotor;
        this.targetpassvelocity = passvelocity;
        this.targetampvelocity = ampvelocity;
        this.targetsendbackvelocity = sendbackvelocity;
        this.targetbaseshootvelocity = basevelocity;
        this.tolerance = tolerance;
        this.pidConfig = new PIDConfig(kp, ki, kd, kf);
    }
}
