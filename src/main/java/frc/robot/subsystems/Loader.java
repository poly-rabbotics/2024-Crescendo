package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.SmartPrinter;

public class Loader extends SmartPrinter {

    private static final double P = 0.05;
    private static final double I = 0;
    private static final double D = 0;

    private static final double COUNTS_PER_REVOLUTION = 70;

    private CANSparkMax loaderMotor;
    private SparkPIDController pidController;

    private double targetPosition = 0;
    
    public Loader(int motorID) {
        super();

        loaderMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        pidController = loaderMotor.getPIDController();

        pidController.setP(P, 0);
        pidController.setI(I, 0);
        pidController.setD(D, 0);

    }

    public void init() {
        loaderMotor.getEncoder().setPosition(0);
    }

    /**
     * Runs the loader's PID loop
     */
    public void run() {
        pidController.setReference(targetPosition * COUNTS_PER_REVOLUTION, ControlType.kPosition, 0);
    }

    /**
     * If true, increment shooter target position by one revolution
     * @param buttonPressed Fire button pressed
     */
    public void fire(boolean buttonPressed) {
        if(buttonPressed) {
            targetPosition += COUNTS_PER_REVOLUTION;
        }
    }

    /**
     * Increment shooter target position by one revolution, then return StepStatus.Done once position is reached
     * @return StepStatus of loader
     */
    public StepStatus fire() {
        StepStatus status = StepStatus.Running;

        if(Math.abs(getEncoderPosition() - targetPosition) > 2) {
            status =  StepStatus.Running;
        } else {
            targetPosition = COUNTS_PER_REVOLUTION;
        }

        if(Math.abs(getEncoderPosition() - targetPosition) < 2) {
            status = StepStatus.Done;
        }

        return status;
    }

    public double getEncoderPosition() {
        return loaderMotor.getEncoder().getPosition() / COUNTS_PER_REVOLUTION;
    }
}
