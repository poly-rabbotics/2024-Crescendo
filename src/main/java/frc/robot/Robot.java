// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import frc.robot.subsystems.Angle;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.ColorUtils;
import frc.robot.subsystems.SidewalkPaver;
import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.PathPosition;
import frc.robot.subsystems.Intake;
import frc.robot.systems.*;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Hands.ShooterState;

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

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Pigeon.setFeildZero();
        Limelight.setPipeline(Limelight.LIMELIGHT_PIPELINE_APRILTAGS_SPEAKERS);
        LEDLights.setBitArrangements(new ColorUtils.BitArrangement[] {
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
            ColorUtils.BitArrangement.GRB,
        });
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

        var procedure = new AutonomousProcedure("My Procedure")
            .wait((prevState) -> {
                intakePath.nextPoseIfComplete(SwerveDrive.getOdometryPose());
                return intakePath.isComplete()
                    ? AutonomousProcedure.StepStatus.Done
                    : AutonomousProcedure.StepStatus.Running;
            })
            .wait((prevState) -> {
                returnPath.nextPoseIfComplete(SwerveDrive.getOdometryPose());
                return returnPath.isComplete()
                    ? AutonomousProcedure.StepStatus.Done
                    : AutonomousProcedure.StepStatus.Running;
            })
            .wait((prevState) -> Hands.pivot.setSetpoint(Setpoint.STATIC_SHOOTING)) //Fucking pivot to shooting position
            .wait((prevState) -> Hands.shooter.setState(ShooterState.RAMPING))
            .wait((prevState) -> Hands.loader.fire());
            
        SwerveDrive.setMode(SwerveMode.SIDEWALK_WALK);
    }

    @Override
    public void autonomousPeriodic() {
        SwerveDrive.run();
        Hands.autoRun();
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
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
            double angle = -((double)pov + 90.0);

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
        SwerveDrive.run(
            controllerOne.getLeftX(),
            -controllerOne.getLeftY(),
            controllerOne.getRightX()
        );

        double rumble = controllerOne.getYButton() 
            ? SwerveDrive.getAverageMotorTemp() / 80.0
            : SwerveDrive.getAveragePercentRatedCurrent();
        controllerOne.setRumble(RumbleType.kBothRumble, rumble);

        /* MECHANISMS CONTROLS TO CHANGE BACK TO AFTER ITS FINISHED
            Hands.run(
            controlPanel.getRawButton(9) || controllerOne.getAButton(),        // Intake
            controlPanel.getRawButton(8),        // Outtake
            controlPanel.getRawButton(6),        // Ramp Up
            controlPanel.getRawButtonPressed(7),        // Fire
            controlPanel.getRawAxis(0) > 0,        // Linear Actuator
            controllerTwo.getLeftY(),                   // Manual Shooter input
            controlPanel.getRawAxis(1),                   // Manual Pivot input
            controlPanel.getRawButton(2),        // Source Intake
            controlPanel.getRawButton(1),        // Ground Intake
            controlPanel.getRawButton(4),        // Speaker Shooting
            controlPanel.getRawButton(5),        // Dynamic Shooting
            controlPanel.getRawButton(3)         // Amp Scoring
        );
        */


        Hands.run(
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
        );
        }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
