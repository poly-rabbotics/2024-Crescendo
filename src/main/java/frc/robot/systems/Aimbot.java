package frc.robot.systems;

import frc.robot.subsystems.Angle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Aimbot {
    private static final double TURN_P = 1;
    private static final double TURN_I = 0.0;
    private static final double TURN_D = 0.0;

    private static final PIDController turnPID = new PIDController(TURN_P, TURN_I, TURN_D);

    public static double calculateTurn() {
        Angle pos = Limelight.targetYawOffset();

        if (pos == null) {
            return 0.0;
        }

        double turn = turnPID.calculate(pos.radians(), 0.0);
        SmartDashboard.putNumber("Aimbot Turn PID Output", turn);
        return turn;
    }
}
