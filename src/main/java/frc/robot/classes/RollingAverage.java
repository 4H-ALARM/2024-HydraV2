package frc.robot.classes;

import java.util.LinkedList;

/**
 * The RollingAverage class calculates the average of the most recent inputs (rolling window).
 * It maintains a fixed buffer size and computes the average of the values stored in that buffer.
 */
public class RollingAverage {
    // The maximum size of the buffer (how many values are averaged at a time)
    private final int bufferSize;

    // A linked list to hold the values in the rolling window
    private final LinkedList<Double> buffer;

    /**
     * Constructor to initialize the RollingAverage with a specified buffer size.
     *
     * @param bufferSize The maximum number of elements to store for the rolling average.
     */
    public RollingAverage(int bufferSize) {
        this.bufferSize = bufferSize;  // Set the buffer size limit
        this.buffer = new LinkedList<>();  // Initialize the buffer to hold the values
    }

    /**
     * Adds a new input value to the rolling average calculation.
     * If the buffer exceeds its size, the oldest value is removed to maintain the buffer size.
     *
     * @param input The new input value to add to the rolling average.
     */
    public void addInput(double input) {
        // If the buffer is full (reached the specified size), remove the oldest (last) element
        if (this.buffer.size() >= this.bufferSize) {
            this.buffer.removeLast();
        }

        // Add the new input value to the front of the buffer
        this.buffer.addFirst(input);
    }

    /**
     * Computes and returns the current rolling average of the values in the buffer.
     *
     * @return The current average of the values stored in the buffer.
     */
    public double getOutput() {
        // Calculate the sum of all elements in the buffer
        double sum = 0.0;
        for (double value : this.buffer) {
            sum += value;
        }

        // Calculate the average by dividing the sum by the number of elements in the buffer
        double average = sum / this.buffer.size();
        return average;
    }
}
