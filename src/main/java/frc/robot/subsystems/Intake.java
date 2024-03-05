package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Intake {

    public static final double INTAKE_SPEED = -0.7;
    public static final double OUTTAKE_SPEED = 0.7;

    private final CANSparkMax outerMotor;
    private final VictorSP innerMotor;

    private double speed = 0;

    public Intake(int outerMotorCanID, int innerMotorPWMChannel) {
        super();

        outerMotor = new CANSparkMax(outerMotorCanID, MotorType.kBrushless);
        innerMotor = new VictorSP(innerMotorPWMChannel);

        outerMotor.setSmartCurrentLimit(25);
        
    }

    /**
     * Runs both intake motors at set speed, call periodically in autonomous or teleop
     */
    public void run() {
        outerMotor.set(speed);
        innerMotor.set(-speed * 1.4);
    }

    /**
     * Sets the speed of the intake motors
     * @param speed
     */
    public void set(double speed) {
        this.speed = speed;
    }

    /**
     * Gets the speed of the inner (VictorSP) motor
     * @return speed
     */
    public double getInnerMotorSpeed() {
        return innerMotor.get();
    }

    /**
     * Gets the speed of the outer (CANSparkMax) motor
     * @return speed
     */
    public double getOuterMotorSpeed() {
        return outerMotor.get();
    }

    /**
     * Gets the intake class's speed value
     * @return speed
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * Gets the temperature of the outer (CANSparkMax) motor
     * @return
     */
    public double getMotorTemperature() {
        return outerMotor.getMotorTemperature();
    }
}
