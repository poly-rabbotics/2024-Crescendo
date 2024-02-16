package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.systems.Hands.ShooterState;

public class Shooter {

    private static final int RAMPING_THRESHOLD = 95; //Threshold where state goes from RAMPING to READY (when i implement it) in percent(?)

    private static double velocity = 120; //Velocity in RPM
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
    public void pidControl(boolean runShooter) {
        if(runShooter) {
            currentState = ShooterState.RUNNING;
        } else {
            currentState = ShooterState.IDLE;
        }

        if(currentState.equals(ShooterState.RUNNING)) {
            leftMotor.setControl(requestLeft.withVelocity(-velocity));
            rightMotor.setControl(requestRight.withVelocity(-velocity));
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    public void manualControl(double speed) {
        currentState = ShooterState.MANUAL;
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    /**
     * Gets the current state of the shooter
     * @return The current state of the shooter
     */
    public ShooterState getState() {
        return currentState;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getSpeed() {
        return leftMotor.getVelocity().getValue();
    }

    public double getOutputPower() {
        return leftMotor.get();
    }
}
