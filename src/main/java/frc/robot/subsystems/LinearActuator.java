package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;

public class LinearActuator {
    private static final double ACTUATION_DISTANCE = 26.5;

    private static final double P = 2.65;
    private static final double I = 0;
    private static final double D = 0;

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
    public LinearActuator(int canSparkId) {
        super();

        motor = new CANSparkMax(canSparkId, MotorType.kBrushless);
        controller = new PIDController(P, I, D);

        motor.setInverted(true);
    }

    public void init() {
        setPosition(0);
    }

    /**
     * Runs the `LinearActuator` setting its motor's speed according to the 
     * current set point and passed-in PID constants.
     */
    public void run() {
        double calculation = controller.calculate(getPosition());

        this.calculation = calculation;
        motor.set(calculation);
    }

    /**
     * Sets the setpoint of the `LinearActuator` and returns StepStatus.Done once position is reached
     * @param setPoint: desired position as a percentage of the actuator's maximum distance
     * @return StepStatus of actuator
     */
    public StepStatus setPosition(double setPoint) {
        StepStatus stepStatus = StepStatus.Running;

        controller.setSetpoint(setPoint);

        if (controller.atSetpoint()) {
            stepStatus = StepStatus.Done;
        }

        return stepStatus;
    }

    /**
     * Gets the position of the `LinearActuator` as a fraction of the actuator's
     * maximum distance. 
     */
    public double getPosition() {
        return motor.getEncoder().getPosition() / ACTUATION_DISTANCE;
    }

    public double getPIDOutput() {
        return calculation;
    }

    public double getMotorTemperature() {
        return motor.getMotorTemperature();
    }
}
