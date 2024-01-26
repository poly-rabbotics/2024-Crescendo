package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.SmartPrintable;

public class Shooter extends SmartPrintable {

    public enum ShooterState {
        IDLE, RUNNING
    }

    private static final int RAMPING_THRESHOLD = 95; //Threshold where state goes from RAMPING to READY (when i implement it) in percent(?)

    private static double velocity = 105; //Velocity in RPM
    private static ShooterState currentState = ShooterState.IDLE;

    private static TalonFX leftMotor;
    private static TalonFX rightMotor;
    private static Slot0Configs slot0Configs;
    private static VelocityVoltage requestLeft;
    private static VelocityVoltage requestRight;
    
    private static final double P_0 = 0.65;
    private static final double I_0 = 0.01;
    private static final double D_0 = 0;
    private static final double S_0 = 0.05;

    public Shooter(int leftMotorID, int rightMotorID) {
        super();

        slot0Configs = new Slot0Configs();
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        slot0Configs.kP = P_0;
        slot0Configs.kI = I_0;
        slot0Configs.kD = D_0;
        slot0Configs.kS = S_0;

        leftMotor.getConfigurator().apply(slot0Configs);
        rightMotor.getConfigurator().apply(slot0Configs);
        requestLeft = new VelocityVoltage(0).withSlot(0);
        requestRight = new VelocityVoltage(0).withSlot(0);
    }

    /**
     * Runs the shooter. What, did you expect it to make you a sandwich?
     * @param runShooter Hold the button to run the shooter
     */
    public void run(boolean runShooter) {
        if(runShooter) {
            currentState = ShooterState.RUNNING;
        } else {
            currentState = ShooterState.IDLE;
        }

        if(currentState.equals(ShooterState.RUNNING)) {
            leftMotor.setControl(requestLeft.withVelocity(velocity));
            rightMotor.setControl(requestRight.withVelocity(-velocity));
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    /**
     * Old and kind of jank run method that lets you change velocity & toggle running
     * @param dPadUp Increments the velocity up by 5
     * @param dPadDown Increments the velocity down by 5
     * @param resume Toggles the shooter on
     * @param stop Toggles the shooter off
     */
    public void runTest(boolean dPadUp, boolean dPadDown, boolean resume, boolean stop) {

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
            leftMotor.setControl(requestLeft.withVelocity(-(velocity*0.75)));
            rightMotor.setControl(requestRight.withVelocity(velocity));
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    /**
     * Gets the current state of the shooter
     * @return The current state of the shooter
     */
    public ShooterState getState() {
        return currentState;
    }

    public void print() {
        SmartDashboard.putString("Shooter State", currentState.toString());
        SmartDashboard.putNumber("Target Velocity", velocity);
        SmartDashboard.putNumber("Motor Speed", leftMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Output Power", leftMotor.get());
    }
}
