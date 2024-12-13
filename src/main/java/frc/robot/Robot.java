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
    private static final XboxController controllerOne = new XboxController(0);
    private static final XboxController controllerTwo = new XboxController(1);
    private static final Joystick controlPanel = new Joystick(2);
    private static final Joystick switchPanel = new Joystick(3);
    private static AutonomousProcedure procedure;
    private static boolean invertedTurn = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        //https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md#new-projects
        Logger.recordMetadata("ProjectName", "7042 Prototype");

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
    }
    
    @Override
    public void robotPeriodic() {
        Pigeon.update();
        Pigeon.recordState();

        SwerveDrive.updateOdometry();
        SwerveDrive.recordStates();

        if (controllerOne.getBackButtonReleased()) {
            var pose = DriverStation.isTeleop() 
                ? new Pose2d(0.0, 0.0, new Rotation2d(0.0))
                : AutonomousManager.getStartingPosSwitches(
                    switchPanel.getRawButton(1),
                    switchPanel.getRawButton(2),
                    switchPanel.getRawButton(3),
                    switchPanel.getRawButton(4)
                );
            
                
            if (switchPanel.getRawButton(6)) {
                SwerveDrive.setTrajectoryCoefficiants(1.0, -1.0);
            } else {
                SwerveDrive.setTrajectoryCoefficiants(1.0, 1.0);
            }

            pose = new Pose2d(
                pose.getX() * SwerveDrive.getTrajectoryCoefficiantX(),
                pose.getY() * SwerveDrive.getTrajectoryCoefficiantY(),
                SwerveDrive.getTrajectoryCoefficiantY() < 0.0
                    ? new Rotation2d(Angle.TAU - pose.getRotation().getRadians())
                    : pose.getRotation()
            );

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
    }

    @Override
    public void autonomousPeriodic() {
        AutonomousManager.getProcedureFromSwitches(
            switchPanel.getRawButton(1),
            switchPanel.getRawButton(2),
            switchPanel.getRawButton(3),
            switchPanel.getRawButton(4)
        ).run();
        
        SwerveDrive.run();
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
    }

    @Override
    public void teleopPeriodic() {
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
                SwerveDrive.setRotationCurve((x) -> Controls.turnCurveRohan(-x));
                invertedTurn = true;
            } else {
                SwerveDrive.setRotationCurve(Controls::turnCurveRohan);
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
        SwerveDrive.conditionalTempMode(SwerveMode.ROCK, controllerOne.getBButton());
        SwerveDrive.conditionalTempMode(SwerveMode.RELATIVE, controllerOne.getYButton());
        SwerveDrive.run(
            -controllerOne.getLeftY(),
            -controllerOne.getLeftX(),
            controllerOne.getRightX()
        );
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
