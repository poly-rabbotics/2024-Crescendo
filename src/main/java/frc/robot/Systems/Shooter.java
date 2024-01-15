package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.SmartPrintable;

public class Shooter extends SmartPrintable {

    public enum ShooterState {
        IDLE, RUNNING
    }
    
    private static final Shooter instance = new Shooter();

    private static final int SHOOTER_MOTOR_ID = 6;
    private static final int RAMPING_THRESHOLD = 95; //Threshold where state goes from RAMPING to READY (when i implement it) in percent(?)

    private static double velocity = 105; //Velocity in RPM
    private static ShooterState currentState = ShooterState.IDLE;

    private static TalonFX shooterMotor;
    private static Slot0Configs slot0Configs;
    private static VelocityVoltage request;
    
    private static final double P_0 = 0.5;
    private static final double I_0 = 0;
    private static final double D_0 = 0;
    private static final double S_0 = 0.05;

    private Shooter() {
        super();

        slot0Configs = new Slot0Configs();
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);

        slot0Configs.kP = P_0;
        slot0Configs.kI = I_0;
        slot0Configs.kD = D_0;
        slot0Configs.kS = S_0;

        shooterMotor.getConfigurator().apply(slot0Configs);
        request = new VelocityVoltage(0).withSlot(0);
    }

    /**
     * Runs the shooter. What, did you expect it to make you a sandwich?
     * @param runShooter Hold the button to run the shooter
     */
    public static void run(boolean runShooter) {
        if(runShooter) {
            currentState = ShooterState.RUNNING;
        } else {
            currentState = ShooterState.IDLE;
        }

        if(currentState.equals(ShooterState.RUNNING)) {
            shooterMotor.setControl(request.withVelocity(velocity));
        } else {
            shooterMotor.set(0);
        }
    }

    /**
     * Old and kind of jank run method that lets you change velocity & toggle running
     * @param dPadUp Increments the velocity up by 5
     * @param dPadDown Increments the velocity down by 5
     * @param resume Toggles the shooter on
     * @param stop Toggles the shooter off
     */
    public static void runTest(boolean dPadUp, boolean dPadDown, boolean resume, boolean stop) {

        if(dPadUp) {
            velocity += 5;
        } else if(dPadDown) {
            velocity -= 5;
        }

        if(resume) {
            currentState = ShooterState.RUNNING;
        } else if(stop) {
            currentState = ShooterState.IDLE;
        }

        
        if(currentState.equals(ShooterState.RUNNING)) {
            shooterMotor.setControl(request.withVelocity(velocity));
        } else {
            shooterMotor.set(0);
        }
    }

    /**
     * Gets the current state of the shooter
     * @return The current state of the shooter
     */
    public static ShooterState getState() {
        return currentState;
    }

    public void print() {
        SmartDashboard.putString("Shooter State", currentState.toString());
        SmartDashboard.putNumber("Target Velocity", velocity);
        SmartDashboard.putNumber("Motor Speed", shooterMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Output Power", shooterMotor.get());
    }
}
