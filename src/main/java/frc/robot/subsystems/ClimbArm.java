package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;

public class ClimbArm {

    private static final double P = 0.05;
    private static final double I = 0.0;
    private static final double D = 0.0;

    private static final double MAX_VELOCITY = 1;
    
    TalonFX climbMotor;
    PIDController pidController;

    public ClimbArm(int motorID) {
        climbMotor = new TalonFX(motorID);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        pidController = new PIDController(
            P, 
            I, 
            D
        );
    }

    public void init() {
        pidController.setSetpoint(0);
    }

    public void run() {
        double calc = pidController.calculate(getVelocity());
        climbMotor.set(calc);
    }

    /**
     * Sets the target velocity of the PID controller
     * @param setpoint, as a percentage of the motor's maximum velocity
     */
    public void set(double setpoint) {
        pidController.setSetpoint(setpoint * MAX_VELOCITY);
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
}
