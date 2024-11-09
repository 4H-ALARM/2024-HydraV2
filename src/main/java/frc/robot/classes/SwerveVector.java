package frc.robot.classes;

public class SwerveVector {
    public double x;
    public double y;
    public double rot;

    public SwerveVector(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    public SwerveVector multiply(double factor) {
        return new SwerveVector(this.x * factor, this.y * factor, this.rot * factor);
    }

    public SwerveVector add(SwerveVector other) {
        return new SwerveVector(this.x + other.x, this.y + other.y, this.rot + other.rot);
    }
}

