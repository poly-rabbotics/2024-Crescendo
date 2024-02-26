package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.Hands;
import frc.robot.systems.Hands.ShooterState;

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

        outerMotor.setSmartCurrentLimit(25);
        
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

    public void autoRun() {
        if(Hands.shooter.getShooterState().equals(ShooterState.IDLE)) {
            outerMotor.set(INTAKE_SPEED);
            innerMotor.set(INTAKE_SPEED);

        }
    }

    public double getInnerMotorSpeed() {
        return innerMotor.get();
    }

    public double getOuterMotorSpeed() {
        return outerMotor.get();
    }
}
