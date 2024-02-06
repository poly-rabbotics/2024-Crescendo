/*
 * Introducing HANDS:
 * Helpful
 * Actuating
 * Note
 * Delivery
 * System
 * !!!!!!!!!!
 */

package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.LinearActuator;
import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Pivot;
import frc.robot.SmartPrintable;


public class Hands extends SmartPrintable {

    public enum Setpoint {
        SOURCE_INTAKE,
        GROUND_INTAKE,
        STATIC_SHOOTING,
        DYNAMIC_SHOOTING,
        AMP_SCORING
    }

    public enum ShooterState {
        IDLE, 
        RUNNING,
        MANUAL
    }

    private static Hands instance = new Hands();

    private static final int LINEAR_ACTUATOR_ID = 61;
    private static final int LINEAR_SERVO_ID = 9;

    private static final int SHOOTER_LEFT_MOTOR_ID = 5;
    private static final int SHOOTER_RIGHT_MOTOR_ID = 6;

    private static final int LOADER_ID = 60;

    private static final int INNER_INTAKE_MOTOR_PWM_CHANNEL = 1;
    private static final int OUTER_INTAKE_MOTOR_CAN_ID = 62;

    private static final int PIVOT_MOTOR_ID = 13;

    private static final double MANUAL_DEADZONE = 0.2;

    private static LinearActuator linearActuator;
    private static LinearServo linearServo;
    private static Shooter shooter;
    private static Intake intake;
    private static Loader loader;
    private static Pivot pivot;
    
    private Hands() {
        super();

        linearActuator = new LinearActuator(LINEAR_ACTUATOR_ID, 108, 0.1, 0.0, 0.0);
        //linearServo = new LinearServo(LINEAR_SERVO_ID);
        loader = new Loader(LOADER_ID);

        shooter = new Shooter(SHOOTER_LEFT_MOTOR_ID, SHOOTER_RIGHT_MOTOR_ID);
        intake = new Intake(OUTER_INTAKE_MOTOR_CAN_ID, INNER_INTAKE_MOTOR_PWM_CHANNEL);
        pivot = new Pivot(PIVOT_MOTOR_ID);

    }

    public static void run(boolean intakeIn, boolean intakeOut, boolean shoot, boolean runLoader, double actuatorPos, double manualShooter, double manualPivot, boolean sourceIntake, boolean groundIntake, boolean speakerShooting, boolean dynamicShooting, boolean ampScoring) {
        loader.run(runLoader);
        intake.run(intakeIn, intakeOut);

        if(intakeIn || intakeOut) {
            
            shooter.manualControl(intakeIn ? -0.2 : -0.2);
        } else {
            if(manualShooter < MANUAL_DEADZONE) {
                shooter.pidControl(shoot);
            } else {
                shooter.manualControl(manualShooter);
            }
        }

        linearActuator.setPosition(actuatorPos);
        linearActuator.run();

        pivot.manualControl(manualPivot * 0.5);

    }

    @Override
    public void print() {
        //Shooter stuff
        SmartDashboard.putString("Shooter State", shooter.getState().toString());
        SmartDashboard.putNumber("Shooter Velocity", shooter.getVelocity());
        SmartDashboard.putNumber("Shooter Motor Power", shooter.getOutputPower());
        SmartDashboard.putNumber("Shooter Motor Speed", shooter.getSpeed());

        //Loader stuff (stuff implies plural, should it be "thing"?)
        SmartDashboard.putNumber("Loader Position", loader.getEncoderPosition());

        //Intake stuff
        SmartDashboard.putNumber("Inner Intake Speed", intake.getInnerMotorSpeed());
        SmartDashboard.putNumber("Outer Intake Speed", intake.getOuterMotorSpeed());

        //Pivot stuff
        SmartDashboard.putNumber("Pivot Position", pivot.getEncoderPosition());
        SmartDashboard.putNumber("Pivot Motor Power", pivot.getOutputPower());
        SmartDashboard.putString("Pivot Setpoint", pivot.getSetpoint().toString());
        SmartDashboard.putNumber("Pivot Position (Raw)", pivot.getRawPosition());
    }
}
