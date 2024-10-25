package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;
import frc.lib.configs.intakeConfig;
import frc.robot.classes.TunableValue;
import frc.robot.classes.handlers.BeamBreakHandler;
import org.littletonrobotics.junction.Logger; // AdvantageKit Logger for telemetry

public class Intake extends SubsystemBase {
    // Singleton instance
    private static Intake instance = null;

    private final CANSparkMax intakeMotor;
    private final intakeConfig config;
    private final BeamBreakHandler beamBreak;

    public final TunableValue INTAKE_MOTOR_SPEED;

    // Private constructor to enforce singleton
    private Intake(intakeConfig config, BeamBreakHandler beamBreakHandler) {
        this.config = config;
        intakeMotor = new CANSparkMax(this.config.intakeMotorId, CANSparkLowLevel.MotorType.kBrushless);
        beamBreak = beamBreakHandler;

        INTAKE_MOTOR_SPEED = new TunableValue("INTAKE_MOTOR_SPEED", 0, Constants.DEBUG);

        // AdvantageKit logger setup for telemetry
        Logger.recordOutput("Intake/MotorSpeed", intakeMotor.get());
        Logger.recordOutput("Intake/IntakeState", "Inactive"); // Record action
    }

    // Public method to get the singleton instance
    public static Intake getInstance(intakeConfig config, BeamBreakHandler beamBreakHandler) {
        if (instance == null) {
            instance = new Intake(config, beamBreakHandler);
        }
        return instance;
    }

    // Basic intake control methods
    public void pickUpNote() {
        intakeMotor.set(this.config.intakeMotorSpeed);
        Logger.recordOutput("Intake/IntakeState", "IntakingNote"); // Record action
    }

    public void rejectNote() {
        intakeMotor.set(-this.config.intakeMotorSpeed);
        Logger.recordOutput("Intake/IntakeState", "RejectingNote"); // Record action
    }

    public void stop() {
        intakeMotor.set(0);
        Logger.recordOutput("Intake/IntakeState", "Inactive"); // Record action
    }

    // Status checking methods
    public boolean isIntakeStopped() {
        boolean isStopped = intakeMotor.getEncoder().getVelocity() == 0;
        Logger.recordOutput("Intake/IsStopped", isStopped); // Record status
        return isStopped;
    }

    public boolean finishedIntaking() {
        boolean isFinished = beamBreak.isSeen();
        Logger.recordOutput("Intake/FinishedIntaking", isFinished); // Record sensor data
        return isFinished;
    }

    // Periodically log motor status
    @Override
    public void periodic() {
        Logger.recordOutput("Intake/MotorVelocity", intakeMotor.getEncoder().getVelocity());
    }
}
