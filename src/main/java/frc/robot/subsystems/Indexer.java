package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
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
        this.indexerMotor.getPIDController().setP(this.config.pidConfig.kp);
        this.indexerMotor.getPIDController().setI(this.config.pidConfig.ki);
        this.indexerMotor.getPIDController().setD(this.config.pidConfig.kd);
        this.indexerMotor.getPIDController().setFF(this.config.pidConfig.kf);
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
     * Starts the indexer motor at a specified velocity
     */
    public void startIndexer(double velocity) {
        indexerMotor.getPIDController().setReference(velocity, CANSparkBase.ControlType.kVelocity);
        Logger.getInstance().recordOutput("Indexer/IndexerState", "Indexing");
    }


    /**
     * Stops the indexer motor.
     */
    public void stopIndexer() {
        indexerMotor.stopMotor();
        Logger.getInstance().recordOutput("Indexer/IndexerState", "Stopped");
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
