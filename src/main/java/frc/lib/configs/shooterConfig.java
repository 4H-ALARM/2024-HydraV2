package frc.lib.configs;

public class shooterConfig {
    public final int shooterTopMotor;
    public final int shooterBottomMotor;
    public final double targetpassvelocity;
    public final double targetampvelocity;
    public final double targetsendbackvelocity;
    public final double targetshufflevelocity;
    public final double targetbaseshootvelocity;
    public final double tolerance;
    public final double P;
    public final double I;
    public final double D;


    public shooterConfig(int topMotor, int bottomMotor, double passvelocity, double ampvelocity, double sendbackvelocity, double shufflevelocity, double basevelocity, double tolerance, double kp, double ki, double kd) {
        this.shooterTopMotor = topMotor;
        this.shooterBottomMotor = bottomMotor;
        this.targetpassvelocity = passvelocity;
        this.targetampvelocity = ampvelocity;
        this.targetsendbackvelocity = sendbackvelocity;
        this.targetshufflevelocity = shufflevelocity;
        this.targetbaseshootvelocity = basevelocity;
        this.tolerance = tolerance;
        this.P = kp;
        this.I = ki;
        this.D = kd;
    }
}
