// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.*;
import frc.robot.subsystems.Shooter;

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

    private static Shooter shooter;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Pigeon.setFeildZero();
        Limelight.setPipeline(Limelight.LIMELIGHT_PIPELINE_APRILTAGS);
        shooter = new Shooter(0, 1);


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
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.HEADLESS);
    }

    @Override
    public void teleopPeriodic() {
        //ClimbTest.runClimb1(controllerOne.getLeftY() * 0.5);

        shooter.pidControl(controllerOne.getAButton());
        SmartDashboard.putNumber("Shooter thing", shooter.getVelocity());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
