package frc.lib;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.configs.COTSswerveconfig;
import frc.lib.configs.swervemoduleconfig;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 4;

        public static final COTSswerveconfig chosenModule =  //TODO: This must be tuned to specific robot
        COTSswerveconfig.KrakenX60Falcon500MK4i(COTSswerveconfig.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 40.0;
        public static final double angleKI = 0.4;
        public static final double angleKD = 2.5;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.25;
        public static final double driveKI = 0.01;
        public static final double driveKD = 0.1125;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.93;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.200195);
            public static final swervemoduleconfig constants =
                new swervemoduleconfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.155762);
            public static final swervemoduleconfig constants =
                new swervemoduleconfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.370850);
            public static final swervemoduleconfig constants =
                new swervemoduleconfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 43;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.437744);
            public static final swervemoduleconfig constants =
                new swervemoduleconfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Auto {





    }
}
