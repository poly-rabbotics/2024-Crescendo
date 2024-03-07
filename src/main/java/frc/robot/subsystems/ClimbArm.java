package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;

public class ClimbArm {

    private static final double P = 0.05;
    private static final double I = 0.0;
    private static final double D = 0.0;

    private static final double MAX_VELOCITY = 0.5;
    private static final double MANUAL_DEADZONE = 0.3;

    private boolean isInverted;
    private boolean atZero = false;

    private double speed = 0;
    
    TalonFX climbMotor;
    PIDController pidController;

    public ClimbArm(int motorID, boolean isInverted) {
        climbMotor = new TalonFX(motorID);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        pidController = new PIDController(
            P, 
            I, 
            D
        );

        this.isInverted = isInverted;
    }

    public void init() {
        pidController.setSetpoint(0);
        climbMotor.setPosition(0);
    }

    public void run() {
        if(atZero) {
            double calc = pidController.calculate(getPosition());
            climbMotor.set(calc);
        } else {
            climbMotor.set(speed);
        }
    }

    /**
     * Sets the target velocity of the PID controller
     * @param setpoint, as a percentage of the motor's maximum velocity
     */
    public void set(double setpoint) {

        if(Math.abs(setpoint) < MANUAL_DEADZONE) {
            speed = 0;

            if(!atZero) {
                pidController.setSetpoint(getPosition());
                atZero = true;
            }
        } else {
            speed = (isInverted ? -setpoint : setpoint) * MAX_VELOCITY;

            atZero = false;
        }
    }

    /**
     * Gets the target velocity of the PID controller
     * @return target velocity as a percentage of the motor's maximum velocity
     */
    public double getTargetVelocity() {
        return pidController.getSetpoint() / MAX_VELOCITY;
    }

    /**
     * Gets the current velocity of the motor
     * @return current velocity, in rotations per second
     */
    public double getVelocity() {
        return climbMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Returns the relative encoder position of the motor's encoder
     */
    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    public boolean getAtZero() {
        return atZero;
    }
}
