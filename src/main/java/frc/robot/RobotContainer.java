package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeromodButton = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public RobotContainer() {
        AutoBuilder.configureHolonomic(
                s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getRobotChassisSpeeds,
                s_Swerve::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(1, 0, 0), // Translation PID constants
                        new PIDConstants(1, 0, 0), // Rotation PID constants
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
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        zeromodButton.onTrue(new InstantCommand(s_Swerve::resetModulesToAbsolute));

    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("New Auto");
    }

}
