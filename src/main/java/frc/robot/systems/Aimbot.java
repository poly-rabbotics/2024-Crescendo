package frc.robot.systems;

import frc.robot.subsystems.Angle;
import frc.robot.SmartPrintable;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Aimbot extends SmartPrintable {
    private static final double TURN_P = 1.4;
    private static final double TURN_I = 0.0;
    private static final double TURN_D = 0.0;

    private static final double TURN_OUTPUT_CENTERED = 0.1;

    private static final double MOVE_P = 0.001;
    private static final double MOVE_I = 0.0;
    private static final double MOVE_D = 0.0;

    private static final double MOVE_OUTPUT_RANGED = 0.1;

    private static final PIDController turnPID = new PIDController(TURN_P, TURN_I, TURN_D);
    private static final PIDController movePID = new PIDController(MOVE_P, MOVE_I, MOVE_D);

    private static Aimbot instance = new Aimbot();

    private double turnCalculation = 0.0;
    private double moveCalculation = 0.0;
    private double desiredDistance = 0.0;

    private Aimbot() {
        SmartPrinter.register(this);
    }

    /**
     * Calculates a turning speed for the drive to use in order to center the 
     * Limelight target. Assumes that the current Limelight target is the 
     * desired target, this should be verified before hand by the caller.
     */
    public static double calculateTurn() {
        Angle pos = Limelight.targetYawOffset();

        if (pos == null) {
            return 0.0;
        }

        instance.turnCalculation = -turnPID.calculate(pos.radians(), 0.0);
        return instance.turnCalculation;
    }

    /**
     * Calculates the required Y-axis speed to approach the current Limelight
     * target. Assumes that the current Limelight target is the  desired target,
     * this should be verified before hand by the caller.
     */
    public static double calculateMovement() {
        double distance = Limelight.estimateTagDistance();

        if (distance != distance) {
            return 0.0;
        }

        instance.moveCalculation = movePID.calculate(distance, instance.desiredDistance);
        return instance.moveCalculation;
    }

    /**
     * Sets the desired distance to the target, in meters.
     */
    public static void setDesiredDistance(double distance) {
        instance.desiredDistance = distance;
    }

    /**
     * True if the target is centered in view, within a tolerance.
     */
    public static boolean isCentered() {
        return Math.abs(instance.turnCalculation) < TURN_OUTPUT_CENTERED;
    }

    /**
     * True the target is at the currently desired distance, within a tolerance.
     * @return
     */
    public static boolean isRanged() {
        return Math.abs(Limelight.estimateTagDistance() - instance.desiredDistance) < MOVE_OUTPUT_RANGED;
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Aimbot Move PID Output", moveCalculation);
        SmartDashboard.putNumber("Aimbot Turn PID Output", turnCalculation);
        SmartDashboard.putBoolean("Aimbot Is Centered?", isCentered());
        SmartDashboard.putBoolean("Aimbot Is Ranged?", isRanged());
    }
}
