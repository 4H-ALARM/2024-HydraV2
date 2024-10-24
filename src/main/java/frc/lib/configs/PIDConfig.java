package frc.lib.configs;

public class PIDConfig {
    public final double kp;
    public final double ki;
    public final double kd;
    public final double kf;

    public PIDConfig(double p, double i, double d, double f) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
    }

    public PIDConfig(double p, double i, double d) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = 0;
    }
}
