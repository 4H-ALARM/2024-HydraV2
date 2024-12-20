package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.classes.LimelightHelpers;
import frc.robot.classes.PhotonCameraHandler;
import frc.robot.classes.TunableValue;
import frc.robot.classes.swervemodules.SwerveModuleKrakenFalcon;
import frc.lib.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.handlers.GyroHandler;

import org.littletonrobotics.junction.Logger;  // AdvantageKit logger

import com.ctre.phoenix6.hardware.Pigeon2;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.List;
import java.util.Optional;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final SwerveModuleKrakenFalcon[] mSwerveMods;
    private final Pigeon2 gyro;
    private ChassisSpeeds latestRobotRelativeSpeeds;
    PhotonCamera camera1;
    PhotonPoseEstimator photonPoseEstimator;
    private Field2d field;
    Pose3d lastpose;

    public final TunableValue PATHPLANNER_TRANSLATION_P;
    public final TunableValue PATHPLANNER_TRANSLATION_I;
    public final TunableValue PATHPLANNER_TRANSLATION_D;

    public final TunableValue PATHPLANNER_ROTATION_P;
    public final TunableValue PATHPLANNER_ROTATION_I;
    public final TunableValue PATHPLANNER_ROTATION_D;

    public Swerve(PhotonCameraHandler photonCameraHandler1) {
        gyro = new Pigeon2(Constants.pigeonID);
        gyro.clearStickyFaults();
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        this.camera1 = new PhotonCamera(Constants.camera1Config.photonCamera);
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        latestRobotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
        this.lastpose = new Pose3d();




        PATHPLANNER_TRANSLATION_P = new TunableValue("PATHPLANNER_TRANSLATION_P", Constants.SwerveConstants.pathplannertranslationpid.kp, Constants.DEBUG);
        PATHPLANNER_TRANSLATION_I = new TunableValue("PATHPLANNER_TRANSLATION_I", Constants.SwerveConstants.pathplannertranslationpid.ki, Constants.DEBUG);
        PATHPLANNER_TRANSLATION_D = new TunableValue("PATHPLANNER_TRANSLATION_D", Constants.SwerveConstants.pathplannertranslationpid.kd, Constants.DEBUG);

        PATHPLANNER_ROTATION_P = new TunableValue("PATHPLANNER_ROTATION_P", Constants.SwerveConstants.pathplannerrotationpid.kp, Constants.DEBUG);
        PATHPLANNER_ROTATION_I = new TunableValue("PATHPLANNER_ROTATION_I", Constants.SwerveConstants.pathplannerrotationpid.ki, Constants.DEBUG);
        PATHPLANNER_ROTATION_D = new TunableValue("PATHPLANNER_ROTATION_D", Constants.SwerveConstants.pathplannerrotationpid.kd, Constants.DEBUG);

        mSwerveMods = new SwerveModuleKrakenFalcon[]{
                new SwerveModuleKrakenFalcon(0, Constants.SwerveConstants.mod0),
                new SwerveModuleKrakenFalcon(1, Constants.SwerveConstants.mod1),
                new SwerveModuleKrakenFalcon(2, Constants.SwerveConstants.mod2),
                new SwerveModuleKrakenFalcon(3, Constants.SwerveConstants.mod3)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), getPose(), stateStdDevs, visionStdDevs);
        field = new Field2d();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModuleKrakenFalcon mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        // Log telemetry using AdvantageKit
        Logger.getInstance().recordOutput("Swerve/ChassisSpeeds", latestRobotRelativeSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                        new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModuleKrakenFalcon mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

        // Log telemetry of desired module states
        for (int i = 0; i < desiredStates.length; i++) {
            Logger.recordOutput("Swerve/ModuleState" + i, desiredStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModuleKrakenFalcon mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModuleKrakenFalcon mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        double yaw = gyro.getYaw().getValue();
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
        photonPoseEstimator.setLastPose(lastpose);
        Logger.recordOutput("Camera/isconnectedcodeside", camera1.isConnected());
        Logger.recordOutput("Camera/hasTargets", camera1.hasTargets());
        Logger.recordOutput("Camera/Targets", camera1.getLatestResult().getTargets().toString());
        Optional<EstimatedRobotPose> estimatedPoseOpt = photonPoseEstimator.update(camera1.getLatestResult());
        Logger.recordOutput("Camera/shouldUpdate", estimatedPoseOpt.isPresent());
        if (estimatedPoseOpt.isPresent()) {
            EstimatedRobotPose pose = estimatedPoseOpt.get();
            Logger.recordOutput("Camera/tags", (String[]) pose.targetsUsed.toArray());
            Logger.recordOutput("Camera/EstimatedPose", pose.estimatedPose.toPose2d());
            swerveDrivePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            lastpose = pose.estimatedPose;
        }
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
        Logger.recordOutput("Swerve/OdometryPose", swerveOdometry.getPoseMeters());
        Logger.recordOutput("Swerve/PoseEstimatorPose", swerveDrivePoseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Swerve/FieldPose", field);


    }
}
