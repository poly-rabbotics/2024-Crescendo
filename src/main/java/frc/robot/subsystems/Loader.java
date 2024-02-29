package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.SmartPrinter;

public class Loader extends SmartPrinter {

    private static final double P = 0.05;
    private static final double I = 0;
    private static final double D = 0;

    private static final double COUNTS_PER_REVOLUTION = 70;

    private CANSparkMax loaderMotor;
    private PIDController pidController;

    private double targetPosition = 0;
    private double lastTarget = 0;
    
    public Loader(int motorID) {
        super();

        loaderMotor = new CANSparkMax(motorID, MotorType.kBrushless);

        pidController = new PIDController(
            P, 
            I, 
            D
        );

        pidController.setTolerance(0.05);
    }

    public void init() {
        loaderMotor.getEncoder().setPosition(0);
        targetPosition = 0;
        lastTarget = 0;
    }

    /**
     * Runs the loader's PID loop
     */
    public void run() {
        pidController.setSetpoint(targetPosition);
        double speed = pidController.calculate(getEncoderPosition());
        
        loaderMotor.set(speed);
    }

    /**
     * Increment shooter target position by one revolution, then return StepStatus.Done once position is reached
     * @return StepStatus of loader
     */
    public StepStatus fire() {
        StepStatus status = StepStatus.Running;
        
        if(pidController.atSetpoint()) {
            if(targetPosition == lastTarget) {
                targetPosition += 1;
            } else {
                status = StepStatus.Done;
                lastTarget = targetPosition;
            }
        }

        return status;
    }

    public double getEncoderPosition() {
        return loaderMotor.getEncoder().getPosition() / COUNTS_PER_REVOLUTION;
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
