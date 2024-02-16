package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbTest {
    private static TalonFX climb1 = new TalonFX(6);

    public static void runClimb1(double speed) {
        climb1.set(speed);
    }
}
