package frc.robot.classes.swervemodules;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Constants;
import frc.lib.math.Conversions;
import frc.lib.configs.swervemoduleconfig;
import frc.robot.Robot;

/**
 * This class represents a MK4i with a Krakenx60 for adrive motor and a Falcon500 for a steer motor.
 * It includes motor configuration, state handling, and feedforward control for swerve drive functionality.
 */
public class SwerveModuleKrakenFalcon {
    public int moduleNumber; // Identifier for the module (e.g., front-left, front-right, etc.)
    private Rotation2d angleOffset; // Angle offset for the module

    // Motors for controlling the angle and drive of the swerve module
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder; // CANCoder sensor for reading the angle

    // Feedforward object to calculate necessary voltage for driving
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    // Control requests for the drive motor
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0); // Open-loop control (percent output)
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withSlot(0); // Closed-loop velocity control

    // Control requests for the angle motor
    private final PositionVoltage anglePosition = new PositionVoltage(0); // Angle position control

    /**
     * Constructor for the SwerveModuleKrakenFalcon class.
     * Initializes the swerve module by setting up the motors and encoders.
     *
     * @param moduleNumber The unique identifier for the module.
     * @param moduleConstants Configuration constants for the swerve module (motor and encoder IDs, angle offset).
     */
    public SwerveModuleKrakenFalcon(int moduleNumber, swervemoduleconfig moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // Configure the angle encoder with the specified CANCoder ID
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        // Configure the angle motor (TalonFX) with the specified settings
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute(); // Reset the angle motor to the encoder's absolute position

        // Configure the drive motor (TalonFX) with the specified settings
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0); // Initialize the drive motor position to zero
    }

    /**
     * Set the desired state (speed and angle) for the swerve module.
     * Adjusts the state to minimize unnecessary rotation (optimize) and applies control to the motors.
     *
     * @param desiredState The desired state containing speed and angle.
     * @param isOpenLoop If true, uses open-loop control (percent output), otherwise closed-loop (velocity control).
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        // Optimize the desired state to avoid unnecessary rotation
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // Set the angle motor to the desired angle
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));

        // Set the drive motor speed based on the control mode (open-loop or closed-loop)
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Set the speed for the drive motor based on the desired state.
     * Uses open-loop (percent output) or closed-loop (velocity control) depending on the mode.
     *
     * @param desiredState The desired state containing speed.
     * @param isOpenLoop If true, uses open-loop control; otherwise, uses closed-loop control.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            // Open-loop: scale speed to percent output and apply to drive motor
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            // Closed-loop: set velocity control and feedforward for precise speed control
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    /**
     * Get the current angle from the CANCoder.
     *
     * @return The current angle as a Rotation2d object.
     */
    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Resets the angle motor's position to the absolute position from the encoder, considering the angle offset.
     */
    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    /**
     * Get the current state of the swerve module (speed and angle).
     *
     * @return The current SwerveModuleState (speed in meters per second and angle).
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /**
     * Get the current position of the swerve module (distance traveled and angle).
     *
     * @return The current SwerveModulePosition (distance in meters and angle).
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }
}
