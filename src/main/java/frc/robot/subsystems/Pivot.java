package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.Hands.ControlMode;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Aimbot;
import frc.robot.systems.Hands;

public class Pivot {
    private static final double P_0 = 0.02;
    private static final double I_0 = 0.0004;
    private static final double D_0 = 0.0003;

    private static final double CLIMBING_ANGLE = 105;
    private static final double GROUND_INTAKE_ANGLE = -60;
    private static final double SPEAKER_SHOOTING_ANGLE = 45;
    private static final double AMP_SCORING_ANGLE = 85;
    
    private Setpoint setpoint = Setpoint.GROUND_INTAKE;
    private ControlMode controlMode = ControlMode.POSITION;

    //Local variables to be used for position/manual control
    private double manualInput = 0;

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

        pidController.setTolerance(0.5);
    }

    /**
     * Initializes the pivot, sets the control mode to position and the setpoint to ground intake
     */
    public void init() {
        set(Setpoint.GROUND_INTAKE);
        setControlMode(ControlMode.POSITION);
        setManualInput(0);
    }

    /**
     * Runs the pivot motor at a set speed, call periodically in autonomous or teleop
     */
    public void run() {
        double speed = 0;

        if(controlMode == ControlMode.POSITION) {
            speed = pidController.calculate(getPosition());
            pivotMotor.set(speed);
        } else {
            pidController.setSetpoint(getTargetPosition() + manualInput);
            speed = pidController.calculate(getPosition());
        }

        pivotMotor.set(speed);
    }

    /**
     * Sets the setpoint of the pivot, to be used in autonomous modes
     * @param setpoint
     * @return
     */
    public StepStatus set(Setpoint setpoint) {
        StepStatus status;
        double target = 0;

        if(setpoint != this.setpoint) {
            this.setpoint = setpoint;

            switch(setpoint) {
                case CLIMBING:
                    target = CLIMBING_ANGLE;
                    break;
                case GROUND_INTAKE:
                    target = GROUND_INTAKE_ANGLE;
                    break;
                case AMP_SCORING:
                    target = AMP_SCORING_ANGLE;
                    break;
                case STATIC_SHOOTING:
                    target = SPEAKER_SHOOTING_ANGLE;
                    break;
                case DYNAMIC_SHOOTING:
                    var angle = Aimbot.calculateShooterAngle();
                
                    if (angle == null) {
                        target = SPEAKER_SHOOTING_ANGLE;
                    } else {
                        target = Hands.clamp(angle.degrees(), 0, 90);
                    }
                
                    break;
            }

            pidController.setSetpoint(target);
        }

        
        if(pidController.atSetpoint())
            status = StepStatus.Done;
        else    
            status = StepStatus.Running;

        return status;
    }

    /**
     * Sets the manual input power to the pivot motor
     * @param input
     */
    public void setManualInput(double input) {
        manualInput = input;
    }

    /**
     * Sets the control mode of the pivot
     * @param mode
     */
    public void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    /**
     * Gets the current position of the pivot (in degrees because I'm not Evan)
     * @return position
     */
    public double getPosition() {
        var pos = absoluteEncoder.getPosition();

        pos = pos > 270.0
            ? -(360.0 - pos)
            : pos;
        
        return (pos);
    }
    
    /**
     * Gets the Setpoint of the pivot
     * @return setpoint
     */
    public Setpoint getSetpoint() {
        return setpoint;
    }

    /**
     * Gets the output power of the pivot motor
     * @return power
     */
    public double getOutputPower() {
        return pivotMotor.get();
    }

    /**
     * Gets the target position of the pivot
     * @return
     */
    public double getTargetPosition() {
        return pidController.getSetpoint();
    }

    /**
     * Returns the control mode of the pivot
     * @return
     */
    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Returns the manual input power to the pivot motor
     * @return manualInput
     */
    public double getManualInput() {
        return manualInput;
    }

    /**
     * Returns the value of the proximity sensor
     * @return
     */
    public boolean getProxSensorTripped() {
        return proxSensor.get();
    }
}
