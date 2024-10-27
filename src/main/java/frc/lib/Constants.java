package frc.lib;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.configs.*;
import frc.robot.classes.TunableValue;
import org.photonvision.PhotonPoseEstimator;

public final class Constants {
    public static final boolean DEBUG = false;
    public static final double stickDeadband = 0.1;





    public static final class SwerveConstants {
        public static final COTSswerveconfig chosenModule = COTSswerveconfig.KrakenX60Falcon500MK4i(COTSswerveconfig.driveRatios.L1);
        public static final swervemoduleconfig mod0 = new swervemoduleconfig(11, 12, 13, Rotation2d.fromRotations(0.299805)); // Front Left Module
        public static final swervemoduleconfig mod1 = new swervemoduleconfig(21, 22, 23, Rotation2d.fromRotations(0.343018)); // Front Right Module
        public static final swervemoduleconfig mod2 = new swervemoduleconfig(31, 32, 33, Rotation2d.fromRotations(0.128906)); // Back Left Module
        public static final swervemoduleconfig mod3 = new swervemoduleconfig(41, 42, 43, Rotation2d.fromRotations(0.060303)); // Back Right Module
        public static final driveTrainConfig drivetrainconfig =  new driveTrainConfig(Units.inchesToMeters(20.75), Units.inchesToMeters(20.75), chosenModule.wheelCircumference);
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(new Translation2d(drivetrainconfig.wheelBase / 2.0, drivetrainconfig.trackWidth / 2.0), new Translation2d(drivetrainconfig.wheelBase / 2.0, -drivetrainconfig.trackWidth / 2.0), new Translation2d(-drivetrainconfig.wheelBase / 2.0, drivetrainconfig.trackWidth / 2.0), new Translation2d(-drivetrainconfig.wheelBase / 2.0, -drivetrainconfig.trackWidth / 2.0));
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double maxSpeed = 3.93;
        public static final double maxAngularVelocity = 10.0;
        public static final PIDConfig anglemotorPID = new PIDConfig(40, 0.4, 2.5);
        public static final PIDConfig drivemotorPID = new PIDConfig(1.25, 0.01, 0.1125);
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;
        public static final PIDConfig pathplannertranslationpid = new PIDConfig(1, 0, 0);
        public static final PIDConfig pathplannerrotationpid = new PIDConfig(4, 0, 0);
    }


    public static final int pigeonID = 4;
    public static final indexerConfig indexerconfig = new indexerConfig(
            55,
            0.5,
            0.5,
            -0.2,
            0.05,
            0,
            0,  
            1
    );
    public static final shooterConfig shooterconfig = new shooterConfig(
            53,
            54,
            0,
            0,
            -0.1,
            0.35,
            0,
            0.05,
            0,
            10,
            1

    );
    public static final intakeConfig intakeconfig = new intakeConfig(
            50,
            -0.8
    );

    public static final beamBreakConfig beambreakconfig = new beamBreakConfig(
            0
    );


    public static final cameraConfig camera1Config = new cameraConfig(
            "orangepi1",
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            );

    public static armConfig armConfig = new armConfig(
            51,
            52,
            new PIDConfig(1.0, 0.0, 0.0),
            0.51,
            0.45,
            0.661,
            1,
            163.32+180,
            new DigitalInput(2)

    );


}
