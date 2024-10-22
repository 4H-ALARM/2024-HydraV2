package frc.robot.subsystems;

import frc.robot.classes.swervemodules.SwerveModuleKrakenFalcon;
import frc.lib.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.GyroHandler;

import org.littletonrobotics.junction.Logger;  // AdvantageKit logger

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final SwerveModuleKrakenFalcon[] mSwerveMods;
    private final GyroHandler gyro;
    private ChassisSpeeds latestRobotRelativeSpeeds;

    public Swerve() {
        gyro = new GyroHandler(Constants.Swerve.pigeonID);
        gyro.getPigeon().clearStickyFaults();
        latestRobotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        mSwerveMods = new SwerveModuleKrakenFalcon[] {
                new SwerveModuleKrakenFalcon(0, Constants.Swerve.Mod0.constants),
                new SwerveModuleKrakenFalcon(1, Constants.Swerve.Mod1.constants),
                new SwerveModuleKrakenFalcon(2, Constants.Swerve.Mod2.constants),
                new SwerveModuleKrakenFalcon(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), getPose(), stateStdDevs, visionStdDevs);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModuleKrakenFalcon mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        // Log telemetry using AdvantageKit
        Logger.getInstance().recordOutput("Swerve/ChassisSpeeds", latestRobotRelativeSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                        new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        this.latestRobotRelativeSpeeds = speeds;

        for (SwerveModuleKrakenFalcon mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }

        Logger.recordOutput("Swerve/RobotRelativeSpeeds", latestRobotRelativeSpeeds);
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return latestRobotRelativeSpeeds;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModuleKrakenFalcon mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

        // Log telemetry of desired module states
        for (int i = 0; i < desiredStates.length; i++) {
            Logger.recordOutput("Swerve/ModuleState" + i, desiredStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleKrakenFalcon mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleKrakenFalcon mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        double yaw = gyro.getPigeon().getYaw().getValue();
        Logger.recordOutput("Swerve/GyroYaw", yaw);
        return Rotation2d.fromDegrees(yaw);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModuleKrakenFalcon mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());

        // Log odometry and pose estimator data
        Logger.recordOutput("Swerve/OdometryPose", swerveOdometry.getPoseMeters());
        Logger.recordOutput("Swerve/PoseEstimatorPose", swerveDrivePoseEstimator.getEstimatedPosition());

        // Log gyro telemetry
        gyro.logTelemetry();
    }
}
