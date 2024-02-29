package frc.robot.systems;

import frc.robot.subsystems.ClimbArm;

public class Climb {
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
}
