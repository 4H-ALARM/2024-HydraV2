package frc.robot.classes;
/**
 * The ToggleHandler class provides a simple mechanism to toggle a boolean state.
 * It can be used to manage on/off states or any other binary state in a system.
 */
public class ToggleHandler {
    // Variable to store the current state (true or false)
    private boolean state;

    /**
     * Constructor to initialize the ToggleHandler.
     * The state is initialized to false by default.
     */
    public ToggleHandler() {
        this.state = false;  // Initial state is set to false (off)
    }

    /**
     * Method to toggle the state.
     * If the current state is true, it becomes false; if it's false, it becomes true.
     */
    public void toggle() {
        state = !state;  // Inverts the current state
    }

    /**
     * Method to get the current state of the toggle.
     *
     * @return The current state (true or false).
     */
    public boolean get() {
        return state;  // Returns the current state
    }

}
