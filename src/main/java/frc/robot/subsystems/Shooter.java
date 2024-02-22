package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.systems.Hands.ControlMode;
import frc.robot.systems.Hands.ShooterState;

public class Shooter {

    private static final int RAMPING_THRESHOLD = 105; //Threshold where state goes from RAMPING to READY (when i implement it) in percent(?)
    private static final double VELOCITY = 120; //Velocity in RPM

    private static double targetVelocity = 0; //Velocity in RPM
    private static ShooterState shooterState = ShooterState.IDLE;
    private static ControlMode controlMode = ControlMode.POSITION;

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
            targetVelocity = VELOCITY;
        } else {
            targetVelocity = 0;
        }
        if(targetVelocity > 0) {
            leftMotor.setControl(requestLeft.withVelocity(targetVelocity));
            rightMotor.setControl(requestRight.withVelocity(-targetVelocity));
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    /**
     * Manually controls the shooter
     * @param speed
     */
    public void manualControl(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    /**
     * Sets the control mode of the shooter
     * @param mode
     */
    public void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    /**
     * Updates the ShooterState object based on speed of the shooter motor
     */
    public void updateShooterState() {
        if(getVelocity() > RAMPING_THRESHOLD) {
            shooterState = ShooterState.AT_SPEED;
        } else if(getVelocity() < 5) {
            shooterState = ShooterState.RAMPING;
        } else {
            shooterState = ShooterState.IDLE;
        }
    }

    /**
     * Returns the current state of the shooter
     */
    public ShooterState getShooterState() {
        return shooterState;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Returns the target velocity of the shooter, in RPM
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Returns the average velocity of the shooter motors, in RPM
     */
    public double getVelocity() {
        return (leftMotor.getVelocity().getValue() + rightMotor.getVelocity().getValue()) / 2.0;
    }

    /**
     * Returns the average output power of the shooter motors
     */
    public double getOutputPower() {
        return (leftMotor.get() + rightMotor.get()) / 2.0;
    }
}
