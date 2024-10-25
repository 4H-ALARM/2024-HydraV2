package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.shooterConfig;
import frc.robot.classes.TunableValue;
import org.littletonrobotics.junction.Logger;
import frc.lib.Constants;

public class Shooter extends SubsystemBase {

    // Singleton instance
    private static Shooter instance = null;

    // Motors
    private final CANSparkMax shooterTopMotor;
    private final CANSparkMax shooterBottomMotor;

    // Configurations
    public final shooterConfig config;

    public final TunableValue SHOOTER_BASE_VELOCITY;
    public final TunableValue SHOOTER_PASS_VELOCITY;
    public final TunableValue SHOOTER_AMP_VELOCITY;
    public final TunableValue SHOOTER_SHUFFLE_VELOCITY;
    public final TunableValue SHOOTER_SENDBACK_VELOCITY;

    public final TunableValue SHOOTERP;
    public final TunableValue SHOOTERI;
    public final TunableValue SHOOTERD;
    public final TunableValue SHOOTERF;

    // Private constructor for singleton pattern
    private Shooter(shooterConfig config) {
        this.config = config;

        //target velocities
        SHOOTER_BASE_VELOCITY = new TunableValue("SHOOTER_BASE_VELOCITY", config.targetbaseshootvelocity, Constants.DEBUG);
        SHOOTER_PASS_VELOCITY = new TunableValue("SHOOTER_PASS_VELOCITY", config.targetpassvelocity, Constants.DEBUG);
        SHOOTER_AMP_VELOCITY = new TunableValue("SHOOTER_AMP_VELOCITY", config.targetampvelocity, Constants.DEBUG);
        SHOOTER_SHUFFLE_VELOCITY = new TunableValue("SHOOTER_SHUFFLE_VELOCITY", config.targetshufflevelocity, Constants.DEBUG);
        SHOOTER_SENDBACK_VELOCITY = new TunableValue("SHOOTER_SENDBACK_VELOCITY", config.targetsendbackvelocity, Constants.DEBUG);

        //PIDF
        SHOOTERP = new TunableValue("SHOOTER_P", config.pidConfig.kp, Constants.DEBUG);
        SHOOTERI = new TunableValue("SHOOTER_I", config.pidConfig.ki, Constants.DEBUG);
        SHOOTERD = new TunableValue("SHOOTER_D", config.pidConfig.kd, Constants.DEBUG);
        SHOOTERF = new TunableValue("SHOOTER_F", config.pidConfig.kf, Constants.DEBUG);

        // Initialize motors
        shooterTopMotor = new CANSparkMax(this.config.shooterTopMotor, CANSparkBase.MotorType.kBrushless);
        shooterTopMotor.getPIDController().setP(SHOOTERP.get(), 0);
        shooterTopMotor.getPIDController().setI(SHOOTERI.get(), 0);
        shooterTopMotor.getPIDController().setD(SHOOTERD.get(), 0);
        shooterTopMotor.getPIDController().setFF(SHOOTERF.get(), 0);

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
        if (Constants.DEBUG) {
            shooterTopMotor.getPIDController().setP(SHOOTERP.get(), 0);
            shooterTopMotor.getPIDController().setI(SHOOTERI.get(), 0);
            shooterTopMotor.getPIDController().setD(SHOOTERD.get(), 0);
            shooterTopMotor.getPIDController().setFF(SHOOTERF.get(), 0);
        }
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
