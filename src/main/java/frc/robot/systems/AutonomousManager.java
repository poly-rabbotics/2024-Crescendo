package frc.robot.systems;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.PathPosition;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 4;
    private static AutonomousManager instance = new AutonomousManager();
    private AutonomousProcedure[] procedures = new AutonomousProcedure[1 << AUTO_SWITCH_COUNT];
    private Pose2d[] startingPositions = new Pose2d[1 << AUTO_SWITCH_COUNT];

    /*
     * Assume autos start width the amp in the positive x direction.
     */

    private AutonomousManager() {
        // Does nothing, assumes starting forward.
        startingPositions[0] = new Pose2d(0.0, 0.0, new Rotation2d(0));
        procedures[0] = new AutonomousProcedure();
    }

    public static void reset() {
        instance = new AutonomousManager();
    }

    /**
     * Gets an auto procedure by its index.
     */
    public static AutonomousProcedure getAutoProcedure(int mode) {
        try {
            AutonomousProcedure proc = instance.procedures[mode];
            return proc;
        } catch (Exception e) {
            System.out.println("Bad switch value!");            
            return new AutonomousProcedure();
        }
    }

    /**
     * Gets an auto mode by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static AutonomousProcedure getProcedureFromSwitches(boolean...  switches) {
        return getAutoProcedure(getSwitchSelection(switches));
    }

    /**
     * Gets an auto start by its index.
     */
    public static Pose2d getStartingPos(int mode) {
        try {
            Pose2d pose = instance.startingPositions[mode];
            return pose;
        } catch (Exception e) {
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");            
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            return new Pose2d();
        }
    }

    /**
     * Gets an auto start by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static Pose2d getStartingPosSwitches(boolean...  switches) {
        return getStartingPos(getSwitchSelection(switches));
    }

    public static int getSwitchSelection(boolean...  switches) {
        int mode = 0;

        for (int i = 0; i < switches.length; i++) {
            mode += switches[i] ? (1 << i) : 0;
        }

        return mode;
    }

    public static void log(boolean... switches) {
        Logger.recordOutput("Auto Manager Selection", getSwitchSelection(switches));
        Logger.recordOutput("Auto Manager Modes Length", instance.procedures.length);
    }

    private static Function<StepStatus, StepStatus> makeDriveStep(Pose2d pose) {
        return (prevState) -> {
            SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

            if (SwerveDrive.withinPositionTolerance()) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }
}
