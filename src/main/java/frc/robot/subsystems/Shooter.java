package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.Hands.ShooterState;
import frc.robot.systems.Hands;
import frc.robot.systems.Hands.ControlMode;

//TODO: CONVERT TO WPILIB PID LOOP
public class Shooter {

    private static final int RAMPING_THRESHOLD = 65; //Threshold where state goes from RAMPING to READY (when i implement it) in percent(?)
    private static final double VELOCITY = 70; //Velocity in RPM
    
    private static final double P_0 = 0.65;
    private static final double I_0 = 0.01;
    private static final double D_0 = 0;
    private static final double S_0 = 0.05;

    private static TalonFX leftMotor;
    private static TalonFX rightMotor;
    private static Slot0Configs slot0Configs;
    private static VelocityVoltage requestLeft;
    private static VelocityVoltage requestRight;

    private ShooterState shooterState = ShooterState.IDLE;
    private ControlMode controlMode = ControlMode.POSITION;
    private double manualInput = 0;


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
    
    public void init() {
        setControlMode(ControlMode.POSITION);
        set(ShooterState.IDLE);
        setManualInput(0);
    }

    /**
     * Runs the shooter motors at a set velocity, call periodically in autonomous or teleop
     */
    public void run() {

        if(getControlMode().equals(ControlMode.POSITION)) { //Position control
            if(getShooterState().equals(ShooterState.RUNNING)) {
                leftMotor.setControl(requestLeft.withVelocity(-VELOCITY));
                rightMotor.setControl(requestRight.withVelocity(-VELOCITY));
            } else {
                leftMotor.set(0);
                rightMotor.set(0);
            }
        } else { //Manual control
            leftMotor.set(-manualInput);
            rightMotor.set(-manualInput);
        }
    }

    /**
     * Sets the ShooterState and target velocity of the shooter
     * @param state
     * @return StepStatus
     */
    public StepStatus set(ShooterState state) {
        StepStatus status;

        shooterState = state;
        
        if(getVelocity() >= RAMPING_THRESHOLD)
            status = StepStatus.Done;
        else
            status = StepStatus.Running;

        if(getShooterState().equals(ShooterState.IDLE))
            status = StepStatus.Done;

        return status;
    }

    /**
     * Sets manual input power to the shooter motors
     * @param input
     */
    public void setManualInput(double input) {
        manualInput = Hands.clamp(input, -0.7, 0.7);
    }

    /**
     * Sets the control mode of the shooter
     * @param mode
     */
    public void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    /**
     * Returns the current ShooterState of the shooter
     * @return shooterState
     */
    public ShooterState getShooterState() {
        return shooterState;
    }

    /**
     * Returns the current ControlMode of the shooter
     * @return controlMode
     */
    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Returns the velocity of the shooter motors, in RPM
     * @return velocity
     */
    public double getVelocity() {
        return leftMotor.getVelocity().refresh().getValue();
    }

    /**
     * Returns the average output power of the shooter motors
     * @return outputPower
     */
    public double getOutputPower() {
        return (leftMotor.get() + rightMotor.get()) / 2.0;
    }

    /**
     * Return last manual control input value
     * @return manualInput
     */
    public double getManualInput() {
        return manualInput;
    }

    public boolean getAtVelocity() {
        return getVelocity() >= VELOCITY;
    }
}