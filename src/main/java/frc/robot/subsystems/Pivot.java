package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.systems.Hands.ControlMode;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Hands;
import frc.robot.systems.Aimbot;
import frc.robot.systems.Limelight;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;

public class Pivot {
    private static final double P_0 = 0.02;
    private static final double I_0 = 0.0004;
    private static final double D_0 = 0.0003;

    private static final double ENCODER_OFFSET = 0;

    private static final double SOURCE_INTAKE_ANGLE = 60;
    private static final double GROUND_INTAKE_ANGLE = -60;
    private static final double SPEAKER_SHOOTING_ANGLE = 45;
    private static final double AMP_SCORING_ANGLE = 85;
    
    private Setpoint setpoint = Setpoint.GROUND_INTAKE;
    private ControlMode controlMode = ControlMode.POSITION;

    private double targetPosition = GROUND_INTAKE_ANGLE;

    private DigitalInput proxSensor;

    private final CANSparkMax pivotMotor;
    private final PIDController pidController;
    private final SparkAbsoluteEncoder absoluteEncoder;
    
    public Pivot(int motorID) {
        pivotMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        absoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        proxSensor = new DigitalInput(0);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        
        pidController = new PIDController(
            P_0, 
            I_0, 
            D_0
        );

        pidController.setTolerance(0.1);
    }

    public void init() {
        setSetpointAuto(Setpoint.GROUND_INTAKE);
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
                targetPosition = SOURCE_INTAKE_ANGLE;
                break;
            case GROUND_INTAKE:
                targetPosition = GROUND_INTAKE_ANGLE;
                break;
            case AMP_SCORING:
                targetPosition = AMP_SCORING_ANGLE;
                break;
            case STATIC_SHOOTING:
                targetPosition = SPEAKER_SHOOTING_ANGLE;
                break;
            case DYNAMIC_SHOOTING:
                var angle = Aimbot.calculateShooterAngle();
                
                if (angle == null) {
                    targetPosition = SPEAKER_SHOOTING_ANGLE;
                } else {
                    targetPosition = Hands.clamp(angle.degrees(), 0, 90);
                }
                
                break;
        }
        
        calc = pidController.calculate(getPosition(), targetPosition);
        
        pivotMotor.set(calc);
    }

    public void manualControl(double speed) {
        double calc = 0;

        targetPosition += speed;

        calc = pidController.calculate(getPosition(), targetPosition);

        pivotMotor.set(calc);
    }

    public void autoRun() {
        double calc = 0;

        calc = pidController.calculate(getPosition(), targetPosition);
        pivotMotor.set(calc);
    }

    /**
     * Gets the current position of the pivot (in degrees because I'm not Evan)
     */
    public double getPosition() {
        var pos = absoluteEncoder.getPosition();

        pos = pos > 270.0
            ? -(360.0 - pos)
            : pos;
        
        return (pos);
    }

    /**
     * Sets the setpoint of the pivot, to be used in autonomous modes
     * @param setpoint
     * @return
     */
    public StepStatus setSetpointAuto(Setpoint setpoint) {
        StepStatus status;

        if(setpoint != this.setpoint) {
            this.setpoint = setpoint;

            switch(setpoint) {
                case SOURCE_INTAKE:
                    targetPosition = SOURCE_INTAKE_ANGLE;
                    break;
                case GROUND_INTAKE:
                    targetPosition = GROUND_INTAKE_ANGLE;
                    break;
                case AMP_SCORING:
                    targetPosition = AMP_SCORING_ANGLE;
                    break;
                case STATIC_SHOOTING:
                    targetPosition = SPEAKER_SHOOTING_ANGLE;
                    break;
                case DYNAMIC_SHOOTING:
                    var angle = Aimbot.calculateShooterAngle();
                
                    if (angle == null) {
                        targetPosition = SPEAKER_SHOOTING_ANGLE;
                    } else {
                        targetPosition = Hands.clamp(angle.degrees(), 0, 90);
                    }
                
                    break;
            }
        }

        
        if(Math.abs(targetPosition - getPosition()) < 2.0)
            status = StepStatus.Done;
        else    
            status = StepStatus.Running;

        return status;
    }
    
    public Setpoint getSetpoint() {
        return setpoint;
    }

    public double getOutputPower() {
        return pivotMotor.get();
    }

    public void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public boolean getProxSensorTripped() {
        return proxSensor.get();
    }
}
