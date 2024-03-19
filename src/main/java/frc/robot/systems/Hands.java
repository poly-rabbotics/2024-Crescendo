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

import frc.robot.SmartPrintable;
import frc.robot.subsystems.*;

public class Hands extends SmartPrintable {

    public enum Setpoint {
        CLIMBING,
        GROUND_INTAKE,
        STATIC_SHOOTING,
        DYNAMIC_SHOOTING,
        AMP_SCORING
    }

    public enum ShooterState {
        IDLE, 
        RUNNING,
    }

    public enum ControlMode {
        MANUAL,
        POSITION
    }

    private static Hands instance = new Hands();

    private static final int LINEAR_ACTUATOR_ID = 61;

    private static final int SHOOTER_LEFT_MOTOR_ID = 5;
    private static final int SHOOTER_RIGHT_MOTOR_ID = 6;

    private static final int LOADER_ID = 60;

    private static final int INNER_INTAKE_MOTOR_PWM_CHANNEL = 1;
    private static final int OUTER_INTAKE_MOTOR_CAN_ID = 62;

    private static final int PIVOT_MOTOR_ID = 13;

    private static final double MANUAL_DEADZONE = 0.3;

    public static LinearActuator linearActuator;
    public static LinearServo linearServo;
    public static Shooter shooter;
    public static Intake intake;
    public static Loader loader;
    public static Pivot pivot;
    
    private Hands() {
        super();
        
        linearActuator = new LinearActuator(LINEAR_ACTUATOR_ID);
        loader = new Loader(LOADER_ID);

        shooter = new Shooter(SHOOTER_LEFT_MOTOR_ID, SHOOTER_RIGHT_MOTOR_ID);
        intake = new Intake(OUTER_INTAKE_MOTOR_CAN_ID, INNER_INTAKE_MOTOR_PWM_CHANNEL);
        pivot = new Pivot(PIVOT_MOTOR_ID);

    }

    /**
     * Run during teleop and autonomous init, resets subsystem values
     */
    public static void init() {
        loader.init();
        shooter.init();
        pivot.init();
    }

    /**
     * Identical to init but doesn't zero the loader position
     */
    public static void teleopInit() {
        shooter.init();
        pivot.init();
    }

    /**
     * Run all subsystems in teleop mode
     * @param intakeIn
     * @param intakeOut
     * @param shoot
     * @param runLoader
     * @param actuatorPressed
     * @param manualShooter
     * @param manualPivot
     * @param climbing
     * @param groundIntake
     * @param speakerShooting
     * @param dynamicShooting
     * @param ampScoring
     */
    public static void run(boolean intakeIn, boolean intakeOut, boolean shoot, boolean runLoader, boolean actuatorPressed, double manualShooter, double manualPivot, boolean climbing, boolean groundIntake, boolean speakerShooting, boolean dynamicShooting, boolean ampScoring) {

        /* PIVOT */
        //Update pivot control mode
        if(climbing || groundIntake || ampScoring || speakerShooting || dynamicShooting) {
            pivot.setControlMode(ControlMode.POSITION);
        } else if(Math.abs(manualPivot) > MANUAL_DEADZONE) {
            pivot.setControlMode(ControlMode.MANUAL);
        }
        
        //Update pivot target pos/set manual input
        Setpoint setpoint = pivot.getSetpoint();

        if(climbing) {
            setpoint = Setpoint.CLIMBING;
        } else if(groundIntake) {
            setpoint = Setpoint.GROUND_INTAKE;
        } else if(speakerShooting) {
            setpoint = Setpoint.STATIC_SHOOTING;
        } else if(dynamicShooting) {
            setpoint = Setpoint.DYNAMIC_SHOOTING;
        } else if(ampScoring) {
            setpoint = Setpoint.AMP_SCORING;
        }

        pivot.set(setpoint);

        pivot.setManualInput(manualPivot);

        /* SHOOTER */
        //Update shooter control mode
        if(shoot) {
            shooter.setControlMode(ControlMode.POSITION);
        } else if(Math.abs(manualShooter) > MANUAL_DEADZONE || intakeIn || intakeOut) {
            shooter.setControlMode(ControlMode.MANUAL);
        }

        //Set shooter state
        if(shoot) {
            shooter.set(ShooterState.RUNNING);
        } else {
            shooter.set(ShooterState.IDLE);
        }

        //Set shooter manual input
        shooter.setManualInput(manualShooter);


        /* LINEAR ACTUATOR */
        if(actuatorPressed) {
            linearActuator.setPosition(0.55);
        } else {
            linearActuator.setPosition(0);
        }     

        /* INTAKE */
        if(intakeIn) {
            intake.set(Intake.INTAKE_SPEED);
            shooter.setManualInput(-0.1);
        } else if(intakeOut) {
            intake.set(Intake.OUTTAKE_SPEED);
            shooter.setManualInput(0.1);
        } else {
            intake.set(0);
            shooter.setManualInput(0);
        }


        /* LOADER */
        if(runLoader) loader.fire();


        //Run subsystems
        pivot.run();
        shooter.run();
        loader.run();
        intake.run();
        linearActuator.run();
    }

    /**
     * run periodically in autonomous mode, updates all subsystems
     */
    public static void autoRun() {
        if(shooter.getShooterState().equals(ShooterState.IDLE) && pivot.getPosition() < 0 && !pivot.getProxSensorTripped()) {
            shooter.setControlMode(ControlMode.MANUAL);
            shooter.setManualInput(-0.2);
            intake.set(Intake.INTAKE_SPEED);
        } else {
            shooter.setControlMode(ControlMode.POSITION);
            intake.set(0);
        }

        intake.run();
        pivot.run();
        shooter.run();
        loader.run();
        linearActuator.run();
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
        SmartDashboard.putNumber("Shooter Manual Input", shooter.getManualInput());
        SmartDashboard.putNumber("Shooter Motor Power", shooter.getOutputPower());
        SmartDashboard.putNumber("Shooter Velocity (RPM)", shooter.getVelocity());


        //Loader stuff (stuff implies plural, should it be "thing"?)
        SmartDashboard.putNumber("Loader Position", loader.getEncoderPosition());
        SmartDashboard.putNumber("Loader last position", loader.getLastTarget());
        SmartDashboard.putNumber("Loader target position", loader.getTargetPosition());
        SmartDashboard.putBoolean("Loader at target", loader.getAtSetpoint());

        //Intake stuff
        SmartDashboard.putNumber("Intake Inner Speed", intake.getInnerMotorSpeed());
        SmartDashboard.putNumber("Intake Outer Speed", intake.getOuterMotorSpeed());
        SmartDashboard.putNumber("Intake temperature", intake.getMotorTemperature());
        SmartDashboard.putNumber("Intake speed", intake.getSpeed());
        SmartDashboard.putNumber("Intake amp pull", INNER_INTAKE_MOTOR_PWM_CHANNEL);

        //Pivot stuff
        SmartDashboard.putNumber("Pivot Position", pivot.getPosition());
        SmartDashboard.putNumber("Pivot Output Power", pivot.getOutputPower());
        SmartDashboard.putString("Pivot Setpoint", pivot.getSetpoint().toString());
        SmartDashboard.putNumber("Pivot Target", pivot.getTargetPosition().degrees());
        SmartDashboard.putNumber("Pivot Manual Input", pivot.getManualInput());
        SmartDashboard.putString("Pivot Control Mode", pivot.getControlMode().toString());
        SmartDashboard.putBoolean("Pivot Prox Sensor", pivot.getProxSensorTripped());

        //Linear actuator stuff
        SmartDashboard.putNumber("Linear Actuator Position", linearActuator.getPosition());
        SmartDashboard.putNumber("Linear Actuator PID Output", linearActuator.getPIDOutput());
        SmartDashboard.putNumber("Linear Actuator Motor Temp", linearActuator.getMotorTemperature());
    }
}
