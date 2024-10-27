package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Constants;
import frc.robot.classes.PhotonCameraHandler;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.classes.handlers.ToggleHandler;
import frc.robot.commands.commandgroups.AutoShootCommandGroup;
import frc.robot.commands.commandgroups.IntakeCommandGroup;
import frc.robot.commands.commandgroups.PrepareShootCommandGroup;
import frc.robot.commands.indexer.FeedNote;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);




    /* Instatiantion of Triggers */

    private final int LeftYAxis = XboxController.Axis.kLeftY.value;
    private final int LeftXAxis = XboxController.Axis.kLeftX.value;
    private final int RightYAxis = XboxController.Axis.kRightY.value;
    private final int RightXAxis = XboxController.Axis.kRightX.value;

    private final int RightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int LeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;

    private final Trigger pilotRightTrigger = pilot.rightTrigger();
    private final Trigger pilotLeftTrigger = pilot.leftTrigger();
    private final Trigger copilotRightTrigger = copilot.rightTrigger();
    private final Trigger copilotLeftTrigger = copilot.leftTrigger();

    private final Trigger pilotRightBumper = pilot.rightBumper();
    private final Trigger pilotLeftBumper = pilot.leftBumper();
    private final Trigger copilotRightBumper = copilot.rightBumper();
    private final Trigger copilotLeftBumper = copilot.leftBumper();

    private final Trigger pilotyButton = pilot.y();
    private final Trigger pilotaButton = pilot.a();
    private final Trigger copilotaButton = copilot.a();
    private final Trigger copilotbButton = copilot.b();

    private final Trigger copilotPOVup = copilot.povUp();
    private final Trigger copilotPOVleft = copilot.povLeft();
    private final Trigger copilotPOVright = copilot.povRight();
    private final Trigger copilotPOVdown = copilot.povDown();


    private final BeamBreakHandler c_BeamBreak;
    private final PhotonCameraHandler c_PhotonCamera1;

    private final ToggleHandler beamBreakDisable;

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Indexer s_Indexer;
    private final Shooter s_Shooter;

    private final IntakeCommandGroup intakeCommandGroup;
    private final PrepareShootCommandGroup prepareShootCommandGroup;
    private final PrepareShootCommandGroup prepareShootCommandGroupcopilot;
    private final AutoShootCommandGroup autoShootCommandGroup;
    private final FeedNote feedNote;
    private final FeedNote feedNotecopilot;


    public RobotContainer() {

        beamBreakDisable = new ToggleHandler("BEAMBREAK_DISABLE");


        c_BeamBreak = new BeamBreakHandler(Constants.beambreakconfig, beamBreakDisable);
        c_PhotonCamera1 = new PhotonCameraHandler(Constants.camera1Config);

        s_Swerve = new Swerve(c_PhotonCamera1);
        s_Intake = Intake.getInstance(Constants.intakeconfig, c_BeamBreak);
        s_Indexer = Indexer.getInstance(Constants.indexerconfig);
        s_Shooter = Shooter.getInstance(Constants.shooterconfig);

        intakeCommandGroup = new IntakeCommandGroup(s_Intake, s_Indexer, c_BeamBreak);
        prepareShootCommandGroup = new PrepareShootCommandGroup(s_Shooter);
        prepareShootCommandGroupcopilot = new PrepareShootCommandGroup(s_Shooter);
        autoShootCommandGroup = new AutoShootCommandGroup(s_Indexer, s_Shooter, c_BeamBreak);
        feedNote = new FeedNote(s_Indexer);
        feedNotecopilot = new FeedNote(s_Indexer);


        NamedCommands.registerCommand("AutoShoot", autoShootCommandGroup);
        NamedCommands.registerCommand("Rev", prepareShootCommandGroup);
        NamedCommands.registerCommand("Feed", feedNote);
        NamedCommands.registerCommand("Intake", intakeCommandGroup);

        AutoBuilder.configureHolonomic(
                s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getRobotChassisSpeeds,
                s_Swerve::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(s_Swerve.PATHPLANNER_TRANSLATION_P.get(), s_Swerve.PATHPLANNER_TRANSLATION_I.get(), s_Swerve.PATHPLANNER_TRANSLATION_D.get()), // Translation PID constants
                        new PIDConstants(s_Swerve.PATHPLANNER_ROTATION_P.get(), s_Swerve.PATHPLANNER_ROTATION_I.get(), s_Swerve.PATHPLANNER_ROTATION_D.get()), // Rotation PID constants
                        0.393,
                        Units.inchesToMeters(15.36),
                        new ReplanningConfig(true,true)
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                s_Swerve);


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -pilot.getRawAxis(LeftYAxis),
                () -> -pilot.getRawAxis(LeftXAxis),
                () -> pilot.getRawAxis(RightXAxis),
                    pilotLeftBumper::getAsBoolean
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        pilotyButton.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        pilotRightTrigger.whileTrue(prepareShootCommandGroup);
        pilotRightBumper.onTrue(feedNote.withTimeout(1));
        pilotLeftTrigger.onTrue(intakeCommandGroup);
        pilotaButton.whileTrue(new PathPlannerAuto("Center3Note"));

        copilotRightTrigger.whileTrue(prepareShootCommandGroup);

    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("Center3Note");
    }

}
