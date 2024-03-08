package frc.robot.systems;

import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Hands.ShooterState;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 4;
    private static AutonomousManager instance = new AutonomousManager();
    private AutonomousProcedure[] procedures = new AutonomousProcedure[1 << (AUTO_SWITCH_COUNT - 1)];

    private AutonomousManager() {
        procedures[0] = new AutonomousProcedure("Unit Procedure");

        procedures[1] = new AutonomousProcedure("")
            .wait((prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING))
            .wait((prevState) -> Hands.shooter.set(ShooterState.RUNNING))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));
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
