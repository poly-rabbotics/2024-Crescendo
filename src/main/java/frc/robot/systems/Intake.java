package frc.robot.systems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.SmartPrintable;

public class Intake extends SmartPrintable {
    private static final int INNER_MOTOR_PWM_CHANNEL = 1;
    private static final int OUTER_MOTOR_CAN_ID = 62;

    private static final double INTAKE_SPEED = -0.35;
    private static final double OUTTAKE_SPEED = 0.5;

    private final CANSparkMax outerMotor = new CANSparkMax(OUTER_MOTOR_CAN_ID, MotorType.kBrushless);
    private final VictorSP innerMotor = new VictorSP(INNER_MOTOR_PWM_CHANNEL);

    private static final Intake instance = new Intake();

    private static double speed = 0;

    private Intake() {
        super();
    }

    public static void run(boolean intake, boolean outtake) {
        if(intake) {
            speed = INTAKE_SPEED;
        } else if(outtake) {
            speed = OUTTAKE_SPEED;
        } else {
            speed = 0;
        }
        
        instance.outerMotor.set(speed);
        instance.innerMotor.set(speed);
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Intake Inner Motor Speed", innerMotor.get());
        SmartDashboard.putNumber("Intake Outer Motor Speed", outerMotor.get());
    }
}
