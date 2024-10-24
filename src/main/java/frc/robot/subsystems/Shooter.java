package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.shooterConfig;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    // Singleton instance
    private static Shooter instance = null;

    // Motors
    private final CANSparkMax shooterTopMotor;
    private final CANSparkMax shooterBottomMotor;

    // Configurations
    public final shooterConfig config;

    // Private constructor for singleton pattern
    private Shooter(shooterConfig config) {
        this.config = config;

        // Initialize motors
        shooterTopMotor = new CANSparkMax(this.config.shooterTopMotor, CANSparkBase.MotorType.kBrushless);
        shooterTopMotor.getPIDController().setP(config.pidConfig.kp, 0);
        shooterTopMotor.getPIDController().setI(config.pidConfig.ki, 0);
        shooterTopMotor.getPIDController().setD(config.pidConfig.kd, 0);
        shooterTopMotor.getPIDController().setFF(config.pidConfig.kf, 0);

        shooterBottomMotor = new CANSparkMax(this.config.shooterBottomMotor, CANSparkBase.MotorType.kBrushless);
        shooterBottomMotor.follow(shooterTopMotor);
    }

    // Method to get the singleton instance
    public static Shooter getInstance(shooterConfig config) {
        if (instance == null) {
            synchronized (Shooter.class) {
                if (instance == null) {
                    instance = new Shooter(config);
                }
            }
        }
        return instance;
    }

    // Start the shooter motors at a given velocity
    public void startShooter(double velocity) {
        // Set the target velocity for the shooter
        shooterTopMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity, 0);

        // Log velocity setpoint
        Logger.recordOutput("Shooter/Stopped", false);
        Logger.recordOutput("Shooter/TargetVelocity", velocity);
    }

    // Stop the shooter motors
    public void stop() {
        shooterTopMotor.stopMotor();
        shooterBottomMotor.stopMotor();

        // Log that the shooter has stopped
        Logger.recordOutput("Shooter/Stopped", true);
    }

    // Check if the shooter has revved up to the target velocity
    public boolean isRevved(double targetVelocity) {
        double currentVelocity = shooterTopMotor.getEncoder().getVelocity();

        // Log current velocity
        Logger.recordOutput("Shooter/CurrentVelocity", currentVelocity);

        // Check if within tolerance
        return currentVelocity > (targetVelocity - config.tolerance) && currentVelocity < (targetVelocity + config.tolerance);
    }

    // Check if the shooter has completely stopped
    public boolean isShooterStopped() {
        double currentVelocity = shooterTopMotor.getEncoder().getVelocity();

        // Log current velocity for stopped check
        Logger.recordOutput("Shooter/CurrentVelocity", currentVelocity);

        return currentVelocity == 0;
    }

    @Override
    public void periodic() {
        // Log shooter motor velocities every loop iteration
        Logger.recordOutput("Shooter/TopMotorVelocity", shooterTopMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/BottomMotorVelocity", shooterBottomMotor.getEncoder().getVelocity());
    }
}
