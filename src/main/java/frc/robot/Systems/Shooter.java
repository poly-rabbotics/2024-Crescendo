package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class Shooter {
    
    private static final Shooter instance = new Shooter();

    private static TalonFX shooterMotor;
    private static Slot0Configs slot0Configs;
    private static VelocityVoltage request;
    private static boolean stopped = true;

    private static final double P_0 = 0.5;
    private static final double I_0 = 0;
    private static final double D_0 = 0;
    private static final double S_0 = 0.05;

    private static double velocity = 70; //Velocity in RPM

    
    private Shooter() {
        slot0Configs = new Slot0Configs();
        shooterMotor = new TalonFX(6);

        slot0Configs.kP = P_0;
        slot0Configs.kI = I_0;
        slot0Configs.kD = D_0;
        slot0Configs.kS = S_0;

        shooterMotor.getConfigurator().apply(slot0Configs);
        request = new VelocityVoltage(0).withSlot(0);
    }

    public static void run(boolean dPadUp, boolean dPadDown, boolean resume, boolean stop) {

        if(dPadUp) {
            velocity += 5;
        } else if(dPadDown) {
            velocity -= 5;
        }

        if(resume) {
            stopped = false;
        } else if(stop) {
            stopped = true;
        }

        
        if(!stopped) shooterMotor.setControl(request.withVelocity(velocity));
        else shooterMotor.set(0);

        SmartDashboard.putNumber("Target Velocity", velocity);
        SmartDashboard.putNumber("Motor Speed", shooterMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Output Power", shooterMotor.get());
    }
}
