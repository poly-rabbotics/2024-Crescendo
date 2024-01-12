// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.systems.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public enum ControlMode {
        DISABLED,
        AUTONOMOUS,
        TELEOPERATED,
        SAFETY
    }

    public static final XboxController controllerOne = (XboxController)Controls.getControllerByPort(0);

    private static Robot instance;
    
    private ControlMode controlMode = ControlMode.DISABLED;

    /**
     * Exists only to enable static methods to gain access to non static data,
     * OOP fans be damned I just made your class a singleton.
     */
    public Robot() {
        super();
        instance = this;
    }

    public static ControlMode getControlMode() {
        return instance.controlMode;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        controlMode = ControlMode.DISABLED;
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {
        controlMode = ControlMode.AUTONOMOUS;
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        controlMode = ControlMode.TELEOPERATED;
    }

    @Override
    public void teleopPeriodic() {
        Shooter.run(controllerOne.getYButtonPressed(), controllerOne.getXButtonPressed(), controllerOne.getAButtonPressed(), controllerOne.getBButtonPressed());
        //Intake.run(controllerOne.getYButtonPressed(), controllerOne.getXButtonPressed(), controllerOne.getAButtonPressed(), controllerOne.getBButtonPressed());
    }

    @Override
    public void disabledInit() {
        controlMode = ControlMode.DISABLED;
    }

    @Override
    public void disabledPeriodic() {}
}
