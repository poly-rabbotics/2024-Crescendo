// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Angle;
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
    private static final Joystick controlPanel = (Joystick)Controls.getControllerByPort(2);

    private static AddressableLED ledStrip = new AddressableLED(8);
    private static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(16);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setBitTiming(250, 1000, 600, 650);


        ledStrip.start();
    }
    
    @Override
    public void robotPeriodic() {
        SmartPrinter.print();
        //LEDLights.run();

        Color color = new Color(0, 255, 0);

        for(int i = 0; i < ledBuffer.getLength(); i++) {
            if(i<4)
                ledBuffer.setLED(i, new Color(0, 255, 0));
            else if(i<8)
                ledBuffer.setLED(i, new Color(100, 255, 0));
            else if(i<12)
                ledBuffer.setLED(i, new Color(255, 10, 10));
            else
                ledBuffer.setLED(i, new Color(0, 0, 255));

        }

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }
    
    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    static boolean invertedTurn = false;

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
