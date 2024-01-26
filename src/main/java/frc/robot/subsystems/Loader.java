package frc.robot.subsystems;

import frc.robot.systems.SmartPrinter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public void run(boolean buttonPressed) {
        if(buttonPressed) {
            targetPosition += COUNTS_PER_REVOLUTION;
        }

        pidController.setReference(targetPosition, ControlType.kPosition, 0);
    }

    public double getEncoderPosition() {
        return loaderMotor.getEncoder().getPosition();
    }
}
