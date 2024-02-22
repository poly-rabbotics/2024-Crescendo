package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.PathPosition;
import frc.robot.subsystems.SidewalkPaver;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 4;
    private static AutonomousManager instance = new AutonomousManager();
    private AutonomousProcedure[] procedures = new AutonomousProcedure[1 << (AUTO_SWITCH_COUNT - 1)];

    private AutonomousManager() {
        procedures[0] = new AutonomousProcedure("Unit Procedure");

        var moveBackPath = new SidewalkPaver(
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))), 
            new PathPosition(new Pose2d(0.0, -1.0, new Rotation2d(Math.toRadians(0.0))), 1.0)
        );

        procedures[1] = new AutonomousProcedure("Preloaded Shooter then Drive")
            .wait((prevState) -> {
                // shoot
                return StepStatus.Done;
            })
            .wait((prevState) -> {
                SwerveDrive.setTargetPathPosition(moveBackPath.nextPoseIfComplete(SwerveDrive.getOdometryPose()));
                return moveBackPath.isComplete()
                    ? StepStatus.Done
                    : StepStatus.Running;
            });
    }

    /**
     * Gets an auto procedure by its index.
     */
    private static AutonomousProcedure getAutoProcedure(int mode) {
        return instance.procedures[mode];
    }

    /**
     * Gets an auto mode by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    private static AutonomousProcedure getProcedureFromSwitches(boolean...  switches) {
        int mode = 0;

        for (int i = 0; i < switches.length; i++) {
            mode += switches[i] ? (1 << i) : 0;
        }

        return getAutoProcedure(mode);
    }
}
