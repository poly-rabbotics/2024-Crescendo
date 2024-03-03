// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import frc.robot.systems.Hands.ShooterState;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.subsystems.*;
import frc.robot.systems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final XboxController controllerOne = (XboxController)Controls.getControllerByPort(0);
    private static final XboxController controllerTwo = (XboxController)Controls.getControllerByPort(1);
    private static final Joystick controlPanel = (Joystick)Controls.getControllerByPort(2);
    AutonomousProcedure procedure;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Pigeon.setFeildZero();
        Limelight.setPipeline(Limelight.LIMELIGHT_PIPELINE_APRILTAGS_SPEAKERS);
        ColorUtils.BitArrangement[] bitArrangements = new ColorUtils.BitArrangement[16];

        for(int i = 0; i < bitArrangements.length; i++) {
            bitArrangements[i] = ColorUtils.BitArrangement.GRB;
        }

        LEDLights.setBitArrangements(bitArrangements);
    }
    
    @Override
    public void robotPeriodic() {
        SmartPrinter.print();
        LEDLights.run();
        Pigeon.update();
        SwerveDrive.updateOdometry();
    }
    
    @Override
    public void autonomousInit() {
        Pigeon.setFeildZero();
        Hands.init();
        
        var intakePath = new SidewalkPaver(
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))), 
            new PathPosition(new Pose2d(1.1, -1.67, new Rotation2d(Math.toRadians(0.0))), 1.0),
            new PathPosition(new Pose2d(3.57, -1.85, new Rotation2d(Math.toRadians(180.0))), 3.0)
        );

        var returnPath = new SidewalkPaver(
            new Pose2d(3.57, -1.85, new Rotation2d(Math.toRadians(180.0))),
            new PathPosition(new Pose2d(0.97, -0.33, new Rotation2d(Math.toRadians(0.0))), 5.0),
            new PathPosition(new Pose2d(0.0, 0.0,new Rotation2d(Math.toRadians(0.0))), 7.0)
        );

        /* Fire straight into shooter and pivot back to ground intake */
        procedure = new AutonomousProcedure("My Procedure")
            .wait((prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)) //Fucking pivot to shooting position
            .wait((prevState) -> Hands.shooter.set(ShooterState.RUNNING)) //Fucking ramp up the shooter
            .wait((prevState) -> Hands.loader.fire()) //Fucking fire the note
            .wait((prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)) //Fucking pivot to ground intake
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE)); //Fucking stop the shooter 
    }

    @Override
    public void autonomousPeriodic() {
        SwerveDrive.run();
        Hands.autoRun();
        procedure.run();
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
        Hands.init();
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

        if (controllerOne.getBackButtonReleased()) {
            SwerveDrive.zeroPositions();
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
            controlPanel.getRawButton(8) || controllerOne.getAButton(),        // Intake
            controlPanel.getRawButton(9),        // Outtake
            controlPanel.getRawButton(6),        // Ramp Up
            controlPanel.getRawButtonPressed(7),        // Fire
            controlPanel.getRawAxis(0) > 0,        // Linear Actuator
            0,                   // Manual Shooter input
            controlPanel.getRawAxis(1),                   // Manual Pivot input
            controlPanel.getRawButton(5),        // Climbing
            controlPanel.getRawButton(1),        // Ground Intake
            controlPanel.getRawButton(3),        // Speaker Shooting
            controlPanel.getRawButton(4),        // Dynamic Shooting
            controlPanel.getRawButton(2)         // Amp Scoring
        );

        Climb.run(
            controllerTwo.getLeftY(), 
            controllerTwo.getRightY()
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
