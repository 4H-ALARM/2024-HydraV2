package frc.robot.classes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableValue {

    private final String key;
    private double defaultValue;
    private boolean tuningMode;

    /**
     * TunableValue.
     * @param key The key used to access the value on SmartDashboard.
     * @param defaultValue The default value to use if tuning is disabled or no dashboard value exists.
     * @param tuningMode If true, the value can be tuned via SmartDashboard.
     */
    public TunableValue(String key, double defaultValue, boolean tuningMode) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.tuningMode = tuningMode;

        // If tuning is enabled, post the default value to the dashboard
        if (tuningMode) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    /**
     * Returns the current value. If tuning is enabled, retrieves it from SmartDashboard;
     * otherwise, it returns the default value.
     *
     * @return The current value.
     */
    public double get() {
        if (tuningMode) {
            return SmartDashboard.getNumber(key, defaultValue);
        } else {
            return defaultValue;
        }
    }

    /**
     * Set a new default value for the TunableValue.
     * Also updates SmartDashboard with the new value if tuning is enabled.
     *
     * @param newValue The new default value.
     */
    public void setDefault(double newValue) {
        this.defaultValue = newValue;

        // Update the dashboard with the new value if in tuning mode
        if (tuningMode) {
            SmartDashboard.putNumber(key, newValue);
        }
    }

    /**
     * Enables or disables tuning mode.
     * When enabled, values will be read from SmartDashboard.
     * When disabled, the default value is used.
     *
     * @param tuningMode True to enable tuning mode; false to disable it.
     */
    public void setTuningMode(boolean tuningMode) {
        this.tuningMode = tuningMode;

        // If switching to tuning mode, update SmartDashboard with the current value
        if (tuningMode) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    /**
     * Returns the key for this TunableValue.
     *
     * @return The SmartDashboard key.
     */
    public String getKey() {
        return key;
    }
}
