package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.lib.Constants;
import frc.lib.CtreConfigs;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.krakenTalonConstants;
import frc.robot.classes.swervemodules.*;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModuleKraken[] mSwerveMods;
    public Pigeon2 gyro;
    private ChassisSpeeds latestRobotRelativeSpeeds;

    public Swerve() {
        gyro = new Pigeon2(Constants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        latestRobotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        mSwerveMods = new SwerveModuleKraken[] {
                new SwerveModuleKraken(0, Constants.mod0frontleftConfig),
                new SwerveModuleKraken(1, Constants.mod1frontrightConfig),
                new SwerveModuleKraken(2, Constants.mod2backleftConfig),
                new SwerveModuleKraken(3, Constants.mod3backrightConfig)
        };

        swerveOdometry = new SwerveDriveOdometry(krakenTalonConstants.Swerve.driveTrainConfig.kinematics, getGyroYaw(), getModulePositions());
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(krakenTalonConstants.Swerve.driveTrainConfig.kinematics, getGyroYaw(), getModulePositions(), getPose(), stateStdDevs, visionStdDevs);

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(3.75, 0.4, 0.085), // Translation PID constants
                        new PIDConstants(3.75, 0.4, 0.085), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                krakenTalonConstants.Swerve.driveTrainConfig.kinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getHeading()
                        )
                                : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, krakenTalonConstants.Swerve.maxSpeed);


        for(SwerveModuleKraken mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.mModule], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        boolean fieldRelative = true;
        double x = speeds.vxMetersPerSecond;
        double y = speeds.vyMetersPerSecond;
        double rot = speeds.omegaRadiansPerSecond;
        SwerveModuleState[] swerveModuleStates =
                krakenTalonConstants.Swerve.driveTrainConfig.kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(
                                x,
                                y,
                                rot)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, krakenTalonConstants.Swerve.maxSpeed);
        this.latestRobotRelativeSpeeds = speeds;
        for(SwerveModuleKraken mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.mModule], true);
        }
    }


    public ChassisSpeeds getRobotChassisSpeeds() {
        return latestRobotRelativeSpeeds;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, krakenTalonConstants.Swerve.maxSpeed);

        for(SwerveModuleKraken mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.mModule], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleKraken mod : mSwerveMods){
            states[mod.mModule] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleKraken mod : mSwerveMods){
            positions[mod.mModule] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }


    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleKraken mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());


        for(SwerveModuleKraken mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.mModule + " CANcoder", mod.getRotation().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.mModule + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.mModule + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}