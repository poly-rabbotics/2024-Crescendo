package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.systems.Hands.Setpoint;


public class Pivot {
    private static final double P_0 = 0.1;
    private static final double I_0 = 0;
    private static final double D_0 = 0;

    private static final double ENCODER_OFFSET = 0;

    private static final double SOURCE_INTAKE_ANGLE = 0;
    private static final double GROUND_INTAKE_ANGLE = 0;
    private static final double SHOOTING_ANGLE = 0;
    private static final double AMP_SCORING_ANGLE = 0;
    
    private Setpoint setpoint = Setpoint.GROUND_INTAKE;

    private final CANSparkMax pivotMotor;
    private final SparkPIDController pidController;
    private final SparkAbsoluteEncoder absoluteEncoder;
    
    public Pivot(int motorID) {
        pivotMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        pidController = pivotMotor.getPIDController();
        absoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        pidController.setP(P_0, 0);
        pidController.setI(I_0, 0);
        pidController.setD(D_0, 0);

        pidController.setFeedbackDevice(absoluteEncoder);
    }

    public void pidControl(boolean sourceIntake, boolean groundIntake, boolean shooting, boolean ampScoring) {
        if(sourceIntake) {
            setpoint = Setpoint.SOURCE_INTAKE;
        } else if(groundIntake) {
            setpoint = Setpoint.GROUND_INTAKE;
        } else if(shooting) {
            setpoint = Setpoint.SHOOTING;
        } else if(ampScoring) {
            setpoint = Setpoint.AMP_SCORING;
        }

        switch(setpoint) {
            case SOURCE_INTAKE:
                pidController.setReference(SOURCE_INTAKE_ANGLE, ControlType.kPosition, 0);
                break;
            case GROUND_INTAKE:
                pidController.setReference(GROUND_INTAKE_ANGLE, ControlType.kPosition, 0);
                break;
            case AMP_SCORING:
                pidController.setReference(AMP_SCORING_ANGLE, ControlType.kPosition, 0);
                break;
            case SHOOTING:
                pidController.setReference(SHOOTING_ANGLE, ControlType.kPosition, 0);
                break;
        }
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
        return (absoluteEncoder.getPosition() - ENCODER_OFFSET) * 360;
    }
}
