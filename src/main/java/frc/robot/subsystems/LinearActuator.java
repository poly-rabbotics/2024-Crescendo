package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.SmartPrintable;

public class LinearActuator extends SmartPrintable {
    private final double actuationDistance;
    private final CANSparkMax motor;
    private final PIDController controller;

    private double calculation = 0.0;

    /**
     * Creates a new `LinearActuator` given a motor controller ID.
     * 
     * @param canSparkId ID of the motor controller
     * @param actuationDistance Actuation distance, in motor rotations, of the 
     * linear actuator.
     */
    public LinearActuator(int canSparkId, double actuationDistance, double p, double i, double d) {
        super();

        motor = new CANSparkMax(canSparkId, MotorType.kBrushless);
        controller = new PIDController(p, i, d);
        this.actuationDistance = actuationDistance;

        motor.setInverted(true);
    }

    /**
     * Gets the position of the `LinearActuator` as a fraction of the actuator's
     * maximun distance.
     */
    public double position() {
        return motor.getEncoder().getPosition() / actuationDistance;
    }

    /**
     * Sets the set point of the `LinearActuator`.
     */
    public void setPosition(double setPoint) {
        controller.setSetpoint(setPoint * actuationDistance);
    }

    /**
     * Runs the `LinearActuator` setting its motor's speed according to the 
     * current set point and passed-in PID constants.
     */
    public void run() {
        double position = motor.getEncoder().getPosition();
        double calculation = controller.calculate(position);

        this.calculation = calculation;
        motor.set(calculation);
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Linear Actuator (on ID " + motor.getDeviceId() + ") distance", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Lineaer Actuator (on ID " + motor.getDeviceId() + ") PID output", calculation);
        SmartDashboard.putNumber("Linear Actuator (on ID" + motor.getDeviceId() + ") Tempurature", motor.getMotorTemperature());
    }
}
