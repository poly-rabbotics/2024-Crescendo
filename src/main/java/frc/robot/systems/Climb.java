package frc.robot.systems;

import frc.robot.SmartPrintable;
import frc.robot.subsystems.ClimbArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SmartPrintable {
    private static Climb instance = new Climb();

    private static final int CLIMB_MOTOR_LEFT_ID = 16;
    private static final int CLIMB_MOTOR_RIGHT_ID = 15;

    private ClimbArm climbLeft;
    private ClimbArm climbRight;
    
    private Climb() {
        super();

        climbLeft = new ClimbArm(CLIMB_MOTOR_LEFT_ID, true);
        climbRight = new ClimbArm(CLIMB_MOTOR_RIGHT_ID, false);
    }

    public static void init() {
        instance.climbLeft.init();
        instance.climbRight.init();
    }

    public static void run(double speedLeft, double speedRight) {
        instance.climbLeft.set(speedLeft);
        instance.climbRight.set(speedRight);

        instance.climbLeft.run();
        instance.climbRight.run();
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Climb Left Target Velocity", climbLeft.getTargetVelocity());
        SmartDashboard.putNumber("Climb Right Target Velocity", climbRight.getTargetVelocity());
        SmartDashboard.putNumber("Climb Left Velocity", climbLeft.getVelocity());
        SmartDashboard.putNumber("Climb Right Velocity", climbRight.getVelocity());
        SmartDashboard.putBoolean("Climb Left at zero", climbLeft.getAtZero());
        SmartDashboard.putBoolean("Climb Right at zero", climbRight.getAtZero());

    }
}
