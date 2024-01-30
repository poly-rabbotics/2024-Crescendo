package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Intake {

    private static final double INTAKE_SPEED = -1.0;
    private static final double OUTTAKE_SPEED = 0.7;

    private final CANSparkMax outerMotor;
    private final VictorSP innerMotor;

    private static double speed = 0;

    public Intake(int outerMotorCanID, int innerMotorPWMChannel) {
        super();

        outerMotor = new CANSparkMax(outerMotorCanID, MotorType.kBrushless);
        innerMotor = new VictorSP(innerMotorPWMChannel);
    }

    public void run(boolean intake, boolean outtake) {
        if(intake) {
            speed = INTAKE_SPEED;
        } else if(outtake) {
            speed = OUTTAKE_SPEED;
        } else {
            speed = 0;
        }
        
        outerMotor.set(speed);
        innerMotor.set(speed);
    }

    public double getInnerMotorSpeed() {
        return innerMotor.get();
    }

    public double getOuterMotorSpeed() {
        return outerMotor.get();
    }
}
