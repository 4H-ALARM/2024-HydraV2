package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limit*/
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimit = 15;
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.anglemotorPID.kp;
        swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.anglemotorPID.ki;
        swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.anglemotorPID.kd;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = 50;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.drivemotorPID.kp;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.drivemotorPID.ki;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.drivemotorPID.kd;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
    }
}