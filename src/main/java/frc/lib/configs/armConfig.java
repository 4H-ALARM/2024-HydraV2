package frc.lib.configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

public class armConfig {
    public final int rightMotorId;
    public final int leftMotorId;

    public final PIDConfig pidconfig;

    public final double shootAngle;
    public final double intakeAngle;
    public final double ampAngle;
    public final double zeroOffset;

    public final double positionScalingFactor;

    public final DigitalInput armLimitSwitch;

    public armConfig(int rightMotor, int leftMotor, PIDConfig pid, double scalingFactor) {
        this(
                rightMotor,
                leftMotor,
                pid,
                0.0, // Default shoot angle
                90.0, // Default intake angle
                60.0, // Default amp angle
                scalingFactor,
                0.0,
                new DigitalInput(2)
        );
    }

    public armConfig(int rightMotor, int leftMotor, PIDConfig pid, double shootAngle, double intakeAngle, double ampAngle, double scalingFactor, double offset, DigitalInput limitSwitch) {
        this.rightMotorId = rightMotor;
        this.leftMotorId = leftMotor;
        this.pidconfig = pid;
        this.shootAngle = shootAngle;
        this.intakeAngle = intakeAngle;
        this.ampAngle = ampAngle;
        this.positionScalingFactor = scalingFactor;
        this.zeroOffset = offset;
        this.armLimitSwitch=limitSwitch;
    }
}
