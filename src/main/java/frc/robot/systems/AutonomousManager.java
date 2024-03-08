package frc.robot.systems;

import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Hands.ShooterState;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 3;
    private static AutonomousManager instance = new AutonomousManager();
    private AutonomousProcedure[] procedures = new AutonomousProcedure[1 << (AUTO_SWITCH_COUNT - 1)];

    private static final AutonomousProcedure scoreSpeaker = new AutonomousProcedure("Speaker Score")
        .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
        .wait((prevState) -> Hands.shooter.set(ShooterState.RUNNING))
        .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.loader.fire()))
        .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
        .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

    private static final AutonomousProcedure scoreAmp = new AutonomousProcedure("Amp Score")
        .wait(AutonomousProcedure.timeoutAt(2, (prevState) -> Hands.pivot.set(Setpoint.AMP_SCORING)))
        .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0.5)))
        .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0.0)))
        .wait(AutonomousProcedure.timeoutAt(0.3, (prevState) -> StepStatus.Running))
        .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0)))
        .wait(AutonomousProcedure.timeoutAt(2, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)));

    private AutonomousManager() {
        procedures[0] = new AutonomousProcedure("Unit Procedure");

        /* MODE 1 // SPEAKER SCORE ONLY */
        procedures[1] = new AutonomousProcedure("Speaker score only")
            .wait(scoreSpeaker);

        /* MODE 2 // DRIVE TO AMP AND SCORE */
        procedures[2] = new AutonomousProcedure("")
            .wait((prevState) -> StepStatus.Running) //SWERVE DRIVING PART
            .wait(scoreAmp);
    }

    /**
     * Gets an auto procedure by its index.
     */
    public static AutonomousProcedure getAutoProcedure(int mode) {
        return instance.procedures[mode];
    }

    /**
     * Gets an auto mode by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static AutonomousProcedure getProcedureFromSwitches(boolean...  switches) {
        int mode = 0;

        for (int i = 0; i < switches.length; i++) {
            mode += switches[i] ? (1 << i) : 0;
        }

        return getAutoProcedure(mode);
    }
}
