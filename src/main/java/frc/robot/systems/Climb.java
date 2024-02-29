package frc.robot.systems;

import frc.robot.SmartPrintable;
import frc.robot.subsystems.ClimbArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SmartPrintable {
    private static Climb instance = new Climb();

    private static final int CLIMB_MOTOR_LEFT_ID = 16;
    private static final int CLIMB_MOTOR_RIGHT_ID = 15;

    private static ClimbArm climbLeft;
    private static ClimbArm climbRight;
    
    private Climb() {
        climbLeft = new ClimbArm(CLIMB_MOTOR_LEFT_ID);
        climbRight = new ClimbArm(CLIMB_MOTOR_RIGHT_ID);
    }

    public static void run(double speedLeft, double speedRight) {
        climbLeft.set(speedLeft);
        climbRight.set(speedRight);

        climbLeft.run();
        climbRight.run();
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Climb Left Target Velocity", climbLeft.getTargetVelocity());
        SmartDashboard.putNumber("Climb Right Target Velocity", climbRight.getTargetVelocity());
        SmartDashboard.putNumber("Climb Left Velocity", climbLeft.getVelocity());
        SmartDashboard.putNumber("Climb Right Velocity", climbRight.getVelocity());
    }
}
