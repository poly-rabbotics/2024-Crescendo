// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.subsystems.*;
import frc.robot.systems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final XboxController controllerOne = (XboxController)Controls.getControllerByPort(0);
    private static final XboxController controllerTwo = (XboxController)Controls.getControllerByPort(1);
    private static final Joystick controlPanel = (Joystick)Controls.getControllerByPort(2);
    private static final Joystick switchPanel = (Joystick)Controls.getControllerByPort(3);
    AutonomousProcedure procedure;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        //https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md#new-projects
        Logger.recordMetadata("ProjectName", "7042 Mantis");

        if (isReal()) {
            // Log for real robot runs
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            new PowerDistribution(1, ModuleType.kRev);
        } else {
            // in simulation mode replay from logs
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        Logger.start();

        // Log driver station data
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Pigeon.setFeildZero();

        Limelight.setPipeline(Limelight.LIMELIGHT_PIPELINE_APRILTAGS_SPEAKERS);

        Hands.init();
        
        ColorUtils.BitArrangement[] bitArrangements = new ColorUtils.BitArrangement[16];
        for(int i = 0; i < bitArrangements.length; i++) { bitArrangements[i] = ColorUtils.BitArrangement.GRB; }
        LEDLights.setBitArrangements(bitArrangements);

        Hands.init();
    }
    
    @Override
    public void robotPeriodic() {
        SmartPrinter.print();
        LEDLights.run();

        Pigeon.update();
        Pigeon.recordState();

        SwerveDrive.updateOdometry();
        SwerveDrive.recordStates();
        
        if (controllerOne.getBackButtonReleased()) {
            // USE SWITCHES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            var pose = DriverStation.isTeleop() 
                ? new Pose2d(0.0, 0.0, new Rotation2d(0.0))
                : AutonomousManager.getStartingPosSwitches(
                    switchPanel.getRawButton(1),
                    switchPanel.getRawButton(2),
                    switchPanel.getRawButton(3),
                    switchPanel.getRawButton(4)
                );
            
            //var pose = AutonomousManager.getStartingPos(4);

            Pigeon.setFeildOrientation(new Angle().setRadians(pose.getRotation().getRadians()));
            SwerveDrive.setOdometry(pose);
        }

        AutonomousManager.log(
            switchPanel.getRawButton(1),
            switchPanel.getRawButton(2),
            switchPanel.getRawButton(3),
            switchPanel.getRawButton(4)
        );
    }
    
    @Override
    public void autonomousInit() {
        SwerveDrive.setMode(SwerveMode.SIDEWALK_WALK);
        AutonomousManager.reset();
        Hands.init();
        Climb.init();
    }

    @Override
    public void autonomousPeriodic() {
        AutonomousManager.getProcedureFromSwitches(
            switchPanel.getRawButton(1),
            switchPanel.getRawButton(2),
            switchPanel.getRawButton(3),
            switchPanel.getRawButton(4)
        ).run();
        
        //AutonomousManager.getAutoProcedure(4).run();
            
        Hands.autoRun();
        SwerveDrive.run();
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
        Hands.init();
        Climb.init();
    }

    static boolean invertedTurn = false;

    @Override
    public void teleopPeriodic() {
        if (controllerOne.getLeftBumper()) {
            Limelight.setPipeline(Limelight.LIMELIGHT_PIPELINE_APRILTAGS_ZOOM);
        } else {
            Limelight.setPipeline(Limelight.LIMELIGHT_PIPELINE_APRILTAGS);
        }

        if (controllerOne.getStartButtonReleased()) {
            Pigeon.setFeildZero();
        }
        
        // Left stick changes between headless and relative control modes.
        if (controllerOne.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(
                SwerveDrive.getMode() == SwerveMode.HEADLESS 
                    ? SwerveMode.RELATIVE 
                    : SwerveMode.HEADLESS
            );
        }

        // Invert turn on right stick.
        if (controllerOne.getRightStickButtonReleased()){       
            if (!invertedTurn) {
                SwerveDrive.setRotationCurve((x) -> Controls.defaultCurve(-x));
                invertedTurn = true;
            } else {
                SwerveDrive.setRotationCurve(Controls::defaultCurve);
                invertedTurn = false;
            }
        }

        double pov = controllerOne.getPOV();

        if (pov != -1 && (SwerveDrive.getMode() == SwerveMode.HEADLESS || SwerveDrive.getMode() == SwerveMode.SET_ANGLE)) {
            double angle = -((double)pov);

            SwerveDrive.setMode(SwerveMode.SET_ANGLE);
            SwerveDrive.setTargetAngle(new Angle().setDegrees(angle));
        } else if (Math.abs(controllerOne.getRightX()) > 0.15 && SwerveDrive.getMode() == SwerveMode.SET_ANGLE) {
            SwerveDrive.setMode(SwerveMode.HEADLESS);
        }
        
        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock(Controls::defaultCurveTwoDimensional), 
            controllerOne.getXButton()
        ); // Lock to cardinal directions.
        SwerveDrive.conditionalTempTranslationCurve(
            (x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0,
            controllerOne.getRightBumper()
        ); // Half translation speed.
        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock((x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 2.0),
            controllerOne.getRightBumper() && controllerOne.getXButton()
        ); // Half translation speed.
        SwerveDrive.conditionalTempMode(SwerveMode.AIMBOT_ROTATION, controllerOne.getLeftTriggerAxis() > 0.5);
        SwerveDrive.conditionalTempMode(SwerveMode.AIMBOT, controllerOne.getRightTriggerAxis() > 0.5);
        SwerveDrive.conditionalTempMode(SwerveMode.ROCK, controllerOne.getBButton());
        SwerveDrive.conditionalTempMode(SwerveMode.RELATIVE, controllerOne.getYButton());
        SwerveDrive.run(
            -controllerOne.getLeftY(),
            -controllerOne.getLeftX(),
            controllerOne.getRightX()
        );

        double rumble = SwerveDrive.getAveragePercentRatedCurrent();
        controllerOne.setRumble(RumbleType.kBothRumble, rumble);

        Hands.run(
            controlPanel.getRawButton(6) || controllerOne.getAButton(),        // Intake
            controlPanel.getRawButton(7),        // Outtake
            controlPanel.getRawButton(10),        // Ramp Up
            controlPanel.getRawButton(11),        // Fire
            controlPanel.getRawButton(8),        // Linear Actuator
            -controlPanel.getRawAxis(1),           // Manual Shooter input
            controlPanel.getRawAxis(0),            // Manual Pivot input
            controlPanel.getRawButton(5),        // Climbing
            controlPanel.getRawButton(1),        // Ground Intake
            controlPanel.getRawButton(3),        // Speaker Shooting
            controlPanel.getRawButton(4),        // Dynamic Shooting
            controlPanel.getRawButton(2)         // Amp Scoring
        );

        Climb.run(
            controllerTwo.getRawAxis(1), 
            controllerTwo.getRawAxis(5)
        );


        /* Hands.run(
            controllerTwo.getLeftBumper() || controllerOne.getAButton(),        // Intake
            controllerTwo.getRightBumper(),        // Outtake
            controllerTwo.getLeftTriggerAxis() > 0.3,        // Ramp Up
            controllerTwo.getAButtonPressed(),        // Fire
            controllerTwo.getLeftStickButton(),        // Linear Actuator
            controllerTwo.getLeftY(),                   // Manual Shooter input
            controllerTwo.getLeftX(),                   // Manual Pivot input
            false,        // Source Intake
            false,        // Ground Intake
            controllerTwo.getXButton(),        // Speaker Shooting
            controllerTwo.getBButton(),        // Dynamic Shooting
            controllerTwo.getYButton()         // Amp Scoring
        ); */
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
