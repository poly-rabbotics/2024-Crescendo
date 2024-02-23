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
        RAMPING,
        AT_SPEED
    }

    public enum ControlMode {
        MANUAL,
        POSITION
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

    public static LinearActuator linearActuator;
    public static LinearServo linearServo;
    public static Shooter shooter;
    public static Intake intake;
    public static Loader loader;
    public static Pivot pivot;
    
    private Hands() {
        super();

        linearActuator = new LinearActuator(LINEAR_ACTUATOR_ID, 26.5, 0.1, 0.0, 0.0);
        //linearServo = new LinearServo(LINEAR_SERVO_ID);
        loader = new Loader(LOADER_ID);

        shooter = new Shooter(SHOOTER_LEFT_MOTOR_ID, SHOOTER_RIGHT_MOTOR_ID);
        intake = new Intake(OUTER_INTAKE_MOTOR_CAN_ID, INNER_INTAKE_MOTOR_PWM_CHANNEL);
        pivot = new Pivot(PIVOT_MOTOR_ID);

    }

    public static void autoInit() {
        loader.init();
        pivot.init();
    }

    public static void run(boolean intakeIn, boolean intakeOut, boolean shoot, boolean runLoader, boolean actuatorPressed, double manualShooter, double manualPivot, boolean sourceIntake, boolean groundIntake, boolean speakerShooting, boolean dynamicShooting, boolean ampScoring) {
        
        //Loader and intake run
        loader.run(runLoader);
        intake.run(intakeIn, intakeOut);

        //Update shooter control mode
        if(shoot) {
            shooter.setControlMode(ControlMode.POSITION);
        } else if(manualShooter > MANUAL_DEADZONE || intakeIn || intakeOut) {
            shooter.setControlMode(ControlMode.MANUAL);
        }

        //Actually running the shooter
        if(shooter.getControlMode().equals(ControlMode.POSITION))
            shooter.pidControl(shoot);
        else if(intakeIn || intakeOut) {
            shooter.manualControl(intakeIn ? -0.25 : 0.25);
        } else {
            shooter.manualControl(manualShooter);
        }

        shooter.updateShooterState();

        //Linear actuator running
        if(actuatorPressed) {
            linearActuator.setPosition(0.45);
        } else {
            linearActuator.setPosition(0);
        }

        linearActuator.run();

        //Update pivot control mode
        if(sourceIntake || groundIntake || ampScoring || speakerShooting || dynamicShooting) {
            pivot.setControlMode(ControlMode.POSITION);
        } else if(manualPivot > MANUAL_DEADZONE) {
            pivot.setControlMode(ControlMode.MANUAL);
        }

        //Pivot running
        if(pivot.getControlMode().equals(ControlMode.POSITION)) {
            pivot.pidControl(sourceIntake, groundIntake, speakerShooting, dynamicShooting, ampScoring);
        } else  {
            pivot.manualControl(manualPivot * 0.2);
        }
    }

    public static void autoRun() {
        intake.autoRun();
        pivot.autoRun();
        shooter.autoRun();
        loader.autoRun();
    }

    /**
     * Clamps num to be between the min and max values
     * @param num
     * @param min
     * @param max
     */
    public static double clamp(double num, double min, double max) {
        if(num < min)
            num = min;
        else if(num > max)
            num = max;

        return num;
    }

    @Override
    public void print() {
        //Shooter stuff
        SmartDashboard.putString("Shooter State", shooter.getShooterState().toString());
        SmartDashboard.putString("Shooter Control Mode", shooter.getControlMode().toString());
        SmartDashboard.putNumber("Shooter Target (RPM)", shooter.getTargetVelocity());
        SmartDashboard.putNumber("Shooter Motor Power", shooter.getOutputPower());
        SmartDashboard.putNumber("Shooter Velocity (RPM)", shooter.getVelocity());

        //Loader stuff (stuff implies plural, should it be "thing"?)
        SmartDashboard.putNumber("Loader Position", loader.getEncoderPosition());

        //Intake stuff
        SmartDashboard.putNumber("Inner Intake Speed", intake.getInnerMotorSpeed());
        SmartDashboard.putNumber("Outer Intake Speed", intake.getOuterMotorSpeed());

        //Pivot stuff
        SmartDashboard.putNumber("Pivot Position", pivot.getPosition());
        SmartDashboard.putNumber("Pivot Motor Power", pivot.getOutputPower());
        SmartDashboard.putString("Pivot Setpoint", pivot.getSetpoint().toString());
        SmartDashboard.putNumber("Pivot Target", pivot.getTargetPosition());
        SmartDashboard.putBoolean("Pivot Prox Sensor", pivot.getProxSensorTripped());

    }
}
