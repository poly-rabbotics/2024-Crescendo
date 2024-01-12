package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class Intake {
    
    private static final Intake instance = new Intake();

    private static TalonFX intakeMotor;

    private static double velocity = 0.9; //Velocity in RPM
    private static boolean stopped = true;

    
    private Intake() {
        intakeMotor = new TalonFX(7);
    }

    public static void run(boolean up, boolean down, boolean resume, boolean stop) {

        if(up) {
            velocity += 0.1;
        } else if(down) {
            velocity -= 0.1;
        }

        if(resume) {
            stopped = false;
        } else if(stop) {
            stopped = true;
        }

        
        if(!stopped) intakeMotor.set(velocity);
        else intakeMotor.set(0);

        SmartDashboard.putNumber("Target Velocity", velocity);
        SmartDashboard.putNumber("Motor Speed", intakeMotor.getVelocity().getValue());
    }
}
