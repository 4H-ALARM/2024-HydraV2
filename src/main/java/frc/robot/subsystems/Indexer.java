package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import org.littletonrobotics.junction.Logger; // AdvantageKit Logger class
import frc.lib.configs.indexerConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final CANSparkMax indexerMotor;
    private final indexerConfig config;

    // Singleton instance
    private static Indexer instance = null;

    // Private constructor for Singleton pattern
    private Indexer(indexerConfig config) {
        this.config = config;
        this.indexerMotor = new CANSparkMax(this.config.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
    }

    /**
     * Get the singleton instance of the Indexer subsystem.
     * @param config The configuration object for the Indexer.
     * @return The singleton instance of the Indexer subsystem.
     */
    public static Indexer getInstance(indexerConfig config) {
        if (instance == null) {
            instance = new Indexer(config);
        }
        return instance;
    }

    /**
     * Starts the indexer motor in reverse to "index" a note.
     */
    public void indexNote() {
        indexerMotor.set(-config.shooterIntakeSpeed);
        Logger.getInstance().recordOutput("Indexer/IndexerState", "Indexing");
    }

    /**
     * Starts the indexer motor at half-speed to "feed" a note.
     */
    public void feedNote() {
        indexerMotor.set(0.5);
        Logger.getInstance().recordOutput("Indexer/IndexerState", "Feeding");
    }

    /**
     * Stops the indexer motor.
     */
    public void stopIndexer() {
        indexerMotor.stopMotor();
        Logger.getInstance().recordOutput("Indexer/IndexerState", "Stopped");
    }

    /**
     * Reverses the indexer motor slightly to send the note back.
     */
    public void sendBack() {
        indexerMotor.set(-0.25);
        Logger.getInstance().recordOutput("Indexer/IndexerState", "Reversing");
    }

    /**
     * Checks if the indexer motor is stopped by checking its velocity.
     * @return True if the motor is stopped, false otherwise.
     */
    public boolean isIndexerStopped() {
        boolean stopped = indexerMotor.getEncoder().getVelocity() == 0;
        Logger.getInstance().recordOutput("Indexer/IsStopped", stopped);
        return stopped;
    }

    /**
     * Periodic function to log the motor velocity and other data.
     */
    @Override
    public void periodic() {
        // Log the indexer motor velocity to AdvantageKit
        double motorVelocity = indexerMotor.getEncoder().getVelocity();
        Logger.getInstance().recordOutput("Indexer/MotorVelocity", motorVelocity);
    }
}
