package frc.lib.configs;

import edu.wpi.first.math.geometry.Rotation2d;

public class moduleConfig {
    public final int driveMotorID;
    public final boolean driveInvert;
    public final int angleMotorID;
    public final boolean angleInvert;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final int modNumber;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param driveInvert
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param modNumber
     */
    public moduleConfig(int driveMotorID, boolean driveInvert, int angleMotorID, boolean angleInvert, int canCoderID, Rotation2d angleOffset, int modNumber) {
        this.driveMotorID = driveMotorID;
        this.driveInvert = driveInvert;
        this.angleMotorID = angleMotorID;
        this.angleInvert = angleInvert;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.modNumber = modNumber;
    }
}
