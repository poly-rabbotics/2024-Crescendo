package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class Shooter {
    
    private static final Shooter instance = new Shooter();

    private static final TalonFX shooterMotor = new TalonFX(6);
    private static Slot0Configs slot0Configs = new Slot0Configs();
    private static VelocityVoltage request;

    private static final double P_0 = 10;
    private static final double I_0 = 0;
    private static final double D_0 = 0;
    private static final double S_0 = 0.05;

    private static double velocity = 3000; //Velocity in RPM

    
    private Shooter() {
        slot0Configs.kP = P_0;
        slot0Configs.kI = I_0;
        slot0Configs.kD = D_0;
        slot0Configs.kS = S_0;

        shooterMotor.getConfigurator().apply(slot0Configs);
        request = new VelocityVoltage(0).withSlot(0);
    }

    public static void run(boolean dPadUp, boolean dPadDown, boolean resume, boolean stop) {

        if(dPadUp) {
            velocity += 100;
            shooterMotor.setControl(request.withVelocity(velocity));
        } else if(dPadDown) {
            velocity -= 100;
            shooterMotor.setControl(request.withVelocity(velocity));
        }

        if(resume) {
            shooterMotor.setControl(request.withVelocity(velocity));
        } else if(stop) {
            shooterMotor.setControl(request.withVelocity(0));
        }
    }
}
