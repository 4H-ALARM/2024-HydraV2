package frc.robot.classes;

import com.ctre.phoenix6.hardware.Pigeon2;
import org.littletonrobotics.junction.Logger; // AdvantageKit Logger for telemetry

public class GyroHandler {
    private final Pigeon2 pigeon;

    // Primary constructor taking CAN ID
    public GyroHandler(int CanID) {
        this(CanID, null);
    }

    // Constructor taking CAN ID and optional CAN bus
    public GyroHandler(int CanID, String Canbus) {
        this.pigeon = new Pigeon2(CanID, Canbus);
        pigeon.clearStickyFaults();
    }

    // Constructor accepting a pre-instantiated Pigeon2 object
    public GyroHandler(Pigeon2 pigeon2) {
        this.pigeon = pigeon2;
        pigeon.clearStickyFaults();
    }

    /**
     * Return the yaw parameter of the gyro
     *
     * @return yaw angle of gyro
     */
    public double getYaw() {
        double yaw = pigeon.getYaw().getValueAsDouble();
        Logger.getInstance().recordOutput("Gyro/Yaw", yaw); // Log with AdvantageKit
        return yaw;
    }

    /**
     * Zeros Gyro
     *
     * @param angle to set the gyro to
     */
    public void zeroGyro(double angle) {
        pigeon.setYaw(angle);
    }


    /**
     * Returns the unwrapped (continuous) gyro angle.
     *
     * @return Unwrapped angle in degrees.
     */
    public double getUnwrappedAngle() {
        double unwrappedAngle = pigeon.getAngle();
        Logger.getInstance().recordOutput("Gyro/UnwrappedAngle", unwrappedAngle); // Log with AdvantageKit
        return unwrappedAngle;
    }

    /**
     * Returns the wrapped angle (0-360 degrees).
     *
     * @return Wrapped angle in degrees.
     */
    public double getWrappedAngle() {
        double unwrappedAngle = pigeon.getAngle();
        double wrappedAngle = unwrappedAngle % 360; // Wrap the angle between 0 and 360
        if (wrappedAngle < 0) {
            wrappedAngle += 360; // Ensure angle is positive
        }
        Logger.getInstance().recordOutput("Gyro/WrappedAngle", wrappedAngle); // Log with AdvantageKit
        return wrappedAngle;
    }

    /**
     * Returns the Pigeon2 object for additional external configuration if needed.
     *
     * @return Pigeon2 instance.
     */
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    /**
     * Updates telemetry for the gyro by logging relevant values.
     * This method can be called periodically from elsewhere in your code.
     */
    public void logTelemetry() {
        // Log both wrapped and unwrapped angles for telemetry
        Logger.getInstance().recordOutput("Gyro/UnwrappedAngle", getUnwrappedAngle());
        Logger.getInstance().recordOutput("Gyro/WrappedAngle", getWrappedAngle());
        Logger.getInstance().recordOutput("Gyro/Yaw", getYaw());
    }
}
