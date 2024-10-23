package frc.robot.classes.handlers;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.configs.beamBreakConfig;

/**
 * The BeamBreakHandler class handles the interaction with a beam break sensor.
 * It uses a digital input sensor to detect when the beam is broken and integrates a toggle mechanism
 * to enable or disable the sensor's reading based on the toggle state.
 */
public class BeamBreakHandler {
    // The digital input object that represents the beam break sensor
    private final DigitalInput colorSensor;

    // Configuration object containing the settings for the beam break sensor
    private final beamBreakConfig config;

    // ToggleHandler object to control when the beam sensor should be active
    private ToggleHandler toggler;

    /**
     * Constructor for the BeamBreakHandler.
     * Initializes the sensor and associates it with a toggle control.
     *
     * @param colorSensorConfig The configuration object for the beam break sensor (contains port info).
     * @param toggle The ToggleHandler to enable or disable the sensor's output reading.
     */
    public BeamBreakHandler(beamBreakConfig colorSensorConfig, ToggleHandler toggle) {
        // Store the configuration object and the toggle handler
        config = colorSensorConfig;
        this.toggler = toggle;

        // Initialize the beam break sensor with the Digital Input channel from the config
        colorSensor = new DigitalInput(config.dio);
    }

    /**
     * Checks whether the beam is broken or not, taking into account the toggle state.
     *
     * @return True if the beam is detected (not broken), false otherwise.
     *         If the toggle is active (true), it will always return false.
     */
    public boolean isSeen() {
        // If the toggle is active, return false (sensor disabled)
        if (this.toggler.get()) {
            return false;
        }
        // Return the sensor's actual value (true if beam is detected, false if broken)
        return colorSensor.get();
    }
}
