package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;


import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Limelight;

/*
 * Reminders to myself on what i need to do before testing this:
 * - Set the motor ID
 * - Set the encoder offset
 * - Find setpoint angles
 * - Add right parameters in Robot.java
 */

public class Pivot {
    private static final double P_0 = 0.02;
    private static final double I_0 = 0;
    private static final double D_0 = 0.0003;

    private static final double ENCODER_OFFSET = 0;

    private static final double SOURCE_INTAKE_ANGLE = 60;
    private static final double GROUND_INTAKE_ANGLE = -60;
    private static final double SPEAKER_SHOOTING_ANGLE = 45;
    private static final double AMP_SCORING_ANGLE = 85;
    
    private Setpoint setpoint = Setpoint.GROUND_INTAKE;

    private final CANSparkMax pivotMotor;
    private final PIDController pidController;
    private final SparkAbsoluteEncoder absoluteEncoder;
    
    public Pivot(int motorID) {
        pivotMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        absoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        pivotMotor.setIdleMode(IdleMode.kBrake);
        
        pidController = new PIDController(
            P_0, 
            I_0, 
            D_0
        );
    }

    public void pidControl(boolean sourceIntake, boolean groundIntake, boolean speakerShooting, boolean dynamicShooting, boolean ampScoring) {
        /* HOLD OPTION */
        if(sourceIntake) {
            setpoint = Setpoint.SOURCE_INTAKE;
        } else if(speakerShooting) {
            setpoint = Setpoint.STATIC_SHOOTING;
        } else if(dynamicShooting) {
            setpoint = Setpoint.DYNAMIC_SHOOTING;
        } else if(ampScoring) {
            setpoint = Setpoint.AMP_SCORING;
        } else {
            setpoint = Setpoint.GROUND_INTAKE;
        }
        
        /* TOGGLE OPTION
        if(sourceIntake) {
            setpoint = Setpoint.SOURCE_INTAKE;
        } else if(groundIntake) {
            setpoint = Setpoint.GROUND_INTAKE;
        } else if(speakerShooting) {
            setpoint = Setpoint.STATIC_SHOOTING;
        } else if(dynamicShooting) {
            setpoint = Setpoint.DYNAMIC_SHOOTING;
        } else if(ampScoring) {
            setpoint = Setpoint.AMP_SCORING;
        } */

        double calc = 0;

        switch(setpoint) {
            case SOURCE_INTAKE:
                calc = pidController.calculate(getEncoderPosition(), SOURCE_INTAKE_ANGLE);
                break;
            case GROUND_INTAKE:
                calc = pidController.calculate(getEncoderPosition(), GROUND_INTAKE_ANGLE);
                break;
            case AMP_SCORING:
                calc = pidController.calculate(getEncoderPosition(), AMP_SCORING_ANGLE);
                break;
            case STATIC_SHOOTING:
                calc = pidController.calculate(getEncoderPosition(), SPEAKER_SHOOTING_ANGLE);
                break;
            case DYNAMIC_SHOOTING:
                calc = pidController.calculate(getEncoderPosition(), Limelight.calculateShooterAngle().degrees());
                break;
        }
        
        pivotMotor.set(calc);
    }

    public void manualControl(double speed) {
        pivotMotor.set(speed);
    }

    public Setpoint getSetpoint() {
        return setpoint;
    }

    public double getOutputPower() {
        return pivotMotor.get();
    }

    /**
     * Gets the current position of the pivot (in degrees because I'm not Evan)
     */
    public double getEncoderPosition() {
        var pos = absoluteEncoder.getPosition();

        pos = pos > 180.0
            ? -(360.0 - pos)
            : pos;
        
        return (pos);
    }

    public double getRawPosition() {
        return absoluteEncoder.getPosition();
    }
}
