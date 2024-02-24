package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Climb {
    private static Climb instance = new Climb();

    private static final int CLIMB_MOTOR_LEFT_ID = 15;
    private static final int CLIMB_MOTOR_RIGHT_ID = 16;

    TalonFX climbMotorLeft;
    TalonFX climbMotorRight;
    
    private Climb() {
        climbMotorLeft = new TalonFX(CLIMB_MOTOR_LEFT_ID);
        climbMotorRight = new TalonFX(CLIMB_MOTOR_RIGHT_ID);
        
        climbMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        climbMotorRight.setNeutralMode(NeutralModeValue.Brake);
    }

    public static void run(double speedLeft, double speedRight) {
        instance.climbMotorLeft.set(speedLeft);
        instance.climbMotorRight.set(speedRight);
    }
}
