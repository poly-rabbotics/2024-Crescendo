// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LinearActuator;
import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.SwerveMode;
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
    private static final LinearActuator linearActuator = new LinearActuator(0, 100, 0.001, 0.0, 0.0);
    private static final LinearServo linearServo = new LinearServo(0);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {}
    
    @Override
    public void robotPeriodic() {
        SmartPrinter.print();
        LEDLights.run();
        
        SmartDashboard.putBoolean("LL Valid Target?", Limelight.hasValidTarget());
        SmartDashboard.putNumber("LL Target April Tag ID", Limelight.aprilTagTargetId());
    }
    
    @Override
    public void autonomousInit() {
        Pigeon.setFeildZero();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
    }

    @Override
    public void teleopPeriodic() {
        // Left stick changes between headless and relative control modes.
        if (controllerOne.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(
                SwerveDrive.getMode() == SwerveMode.HEADLESS 
                    ? SwerveMode.RELATIVE 
                    : SwerveMode.HEADLESS
            );
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

        double rumble = controllerOne.getLeftBumper() 
            ? SwerveDrive.getAverageMotorTemp() / 80.0
            : SwerveDrive.getAveragePercentRatedCurrent();
        controllerOne.setRumble(RumbleType.kBothRumble, rumble);

        linearActuator.setPosition(controllerTwo.getRightTriggerAxis());
        linearServo.setPosition(controllerTwo.getLeftTriggerAxis());

        Intake.run(controllerTwo.getLeftY(), controllerTwo.getRightY());

        Shooter.runTest(controllerOne.getYButtonPressed(), controllerOne.getXButtonPressed(), controllerOne.getAButtonPressed(), controllerOne.getBButtonPressed());
        //Intake.runTest(controllerOne.getYButtonPressed(), controllerOne.getXButtonPressed(), controllerOne.getAButtonPressed(), controllerOne.getBButtonPressed());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
