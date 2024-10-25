package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import frc.lib.Constants;
import frc.robot.classes.TunableValue;
import org.littletonrobotics.junction.Logger; // AdvantageKit Logger class
import frc.lib.configs.indexerConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final CANSparkMax indexerMotor;
    private final indexerConfig config;

    // Singleton instance
    private static Indexer instance = null;

    public final TunableValue INDEXER_FEED_VELOCITY;
    public final TunableValue INDEXER_FEEDBACK_VELOCITY;
    public final TunableValue INDEXER_INTAKE_VELOCITY;

    public final TunableValue INDEXERP;
    public final TunableValue INDEXERI;
    public final TunableValue INDEXERD;
    public final TunableValue INDEXERF;

    // Private constructor for Singleton pattern
    private Indexer(indexerConfig config) {
        this.config = config;

        INDEXER_FEED_VELOCITY = new TunableValue("INDEXER_FEED_VELOCITY", 0, Constants.DEBUG);;
        INDEXER_FEEDBACK_VELOCITY = new TunableValue("INDEXER_FEEDBACK_VELOCITY", 0, Constants.DEBUG);
        INDEXER_INTAKE_VELOCITY = new TunableValue("INDEXER_INTAKE_VELOCITY", 0, Constants.DEBUG);

        //PIDF
        INDEXERP = new TunableValue("INDEXER_P", config.pidConfig.kp, Constants.DEBUG);
        INDEXERI = new TunableValue("INDEXER_I", config.pidConfig.ki, Constants.DEBUG);
        INDEXERD = new TunableValue("INDEXER_D", config.pidConfig.kd, Constants.DEBUG);
        INDEXERF = new TunableValue("INDEXER_F", config.pidConfig.kf, Constants.DEBUG);

        this.indexerMotor = new CANSparkMax(this.config.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
        this.indexerMotor.getPIDController().setP(INDEXERP.get());
        this.indexerMotor.getPIDController().setI(INDEXERI.get());
        this.indexerMotor.getPIDController().setD(INDEXERD.get());
        this.indexerMotor.getPIDController().setFF(INDEXERF.get());
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
        // indexerMotor.getPIDController().setReference(velocity, CANSparkBase.ControlType.kVelocity);
        indexerMotor.set(0.5);
        Logger.recordOutput("Indexer/IndexerState", "Indexing");
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
