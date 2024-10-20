package frc.robot.systems;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.PathPosition;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Hands.ShooterState;

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
        procedures[0] = new AutonomousProcedure("Unit Procedure");

        // Scores speaker with preload, assumes facing speaker from center.
        startingPositions[1] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[1] = new AutonomousProcedure("Speaker 1 Note")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))   
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

        startingPositions[2] = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
        procedures[2] = new AutonomousProcedure("Speaker 1 Note (Amp Side)")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

        // Scores preload, gets front row note and scores it
        // assumes starting center speaker, facing speaker.
        startingPositions[3] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[3] = new AutonomousProcedure("Speaker Score -> Leave -> Intake -> Move Back -> Speaker Score")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(makeDriveStep(new Pose2d(1.25, 0.0, new Rotation2d(0.0))))
            .wait(makeDriveStep(new Pose2d(1.75, 0.0, new Rotation2d(0.0))))
            .wait(makeDriveStep(new Pose2d(1.25, 0.0, new Rotation2d(Math.PI))))
            .wait(makeDriveStep(new Pose2d(0.0, 0.0, new Rotation2d(Math.PI))))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));
        
        // Source side two note
        startingPositions[4] = new Pose2d(0.0, 0.0, new Rotation2d(2.366));
        procedures[4] = new AutonomousProcedure("Speaker Score -> Leave -> Intake -> Move Back -> Speaker Score")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(makeDriveStep(new Pose2d(1.75, -0.15, new Rotation2d(0.0))))
            .wait(AutonomousProcedure.timeoutAt(1.75, makeDriveStep(new Pose2d(2.25, -0.15, new Rotation2d(0.0)))))
            .wait(makeDriveStep(new Pose2d(1.5, -0.15, new Rotation2d(Math.PI))))
            .wait(makeDriveStep(new Pose2d(0.0, 0.0, new Rotation2d(2.366))))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));
        
        // amp side two note
        startingPositions[5] = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
        procedures[5] = new AutonomousProcedure("Amp 2 Note")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(1.0, 0.4, new Rotation2d(0.0)))))
            .wait(AutonomousProcedure.timeoutAt(2.25, makeDriveStep(new Pose2d(3.5, 0.4, new Rotation2d(0.0)))))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(0.0, 0.0, new Rotation2d(-2.237)))))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()));
        
        // Source side - shoot preload, then dine and dash
        startingPositions[6] = new Pose2d(0.0, 0.0, new Rotation2d(2.237));
        procedures[6] = new AutonomousProcedure("Source Side Preload - Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.5, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.0, makeDriveStep(new Pose2d(0.0, -2.25, new Rotation2d(2.237)))))
            .wait(makeDriveStep(new Pose2d(7.2, -2.25, new Rotation2d(Math.PI))));
            
        // Amp side speaker - shoot preload then dine and dash
        startingPositions[7] = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
        procedures[7] = new AutonomousProcedure("Amp Side Preload -> Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.5, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(makeDriveStep(new Pose2d(7.2, -0.14, new Rotation2d(Math.PI))));

        // Source side - two note, then dine and dash
        startingPositions[8] = new Pose2d(0.0, 0.0, new Rotation2d(2.237));
        procedures[8] = new AutonomousProcedure("Source Two Note -> Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.75, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(makeDriveStep(new Pose2d(1.75, -0.15, new Rotation2d(0.0))))
            .wait(AutonomousProcedure.timeoutAt(1.75, makeDriveStep(new Pose2d(2.25, -0.15, new Rotation2d(0.0)))))
            .wait(makeDriveStep(new Pose2d(1.5, -0.15, new Rotation2d(Math.PI))))
            .wait(makeDriveStep(new Pose2d(0.0, 0.0, new Rotation2d(2.366))))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.5, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.0, makeDriveStep(new Pose2d(0.0, -2.25, new Rotation2d(2.237)))))
            .wait(makeDriveStep(new Pose2d(7.2, -2.25, new Rotation2d(Math.PI))));
            
        // Amp side speaker two note then dine and dash
        startingPositions[9] = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
        procedures[9] = new AutonomousProcedure("Amp 2 Note -> Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(1.0, 0.25, new Rotation2d(0.0)))))
            .wait(AutonomousProcedure.timeoutAt(2.25, makeDriveStep(new Pose2d(3.5, 0.25, new Rotation2d(0.0)))))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(0.0, 0.0, new Rotation2d(-2.237)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevStep) -> StepStatus.Running))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(makeDriveStep(new Pose2d(7.2, -0.14, new Rotation2d(Math.PI))));
            
        // Start center speaker, three note auto
        startingPositions[10] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[10] = new AutonomousProcedure("Three Note")
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(2.2, 2.2, new Rotation2d(0.65)))))
            .wait(AutonomousProcedure.timeoutAt(2.0, makeDriveStep(new Pose2d(0.0, 0.2, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(1.8, 1.85, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, makeDriveStep(new Pose2d(4.0, -0.5, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(2.0, makeDriveStep(new Pose2d(0.25, -0.5, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));
        
        // Start center speaker, three note auto, dine and dash
        startingPositions[11] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[11] = new AutonomousProcedure("Three Note -> Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.5, makeDriveStep(new Pose2d(2.0, 1.85, new Rotation2d(0.65)))))
            .wait(AutonomousProcedure.timeoutAt(2.0, makeDriveStep(new Pose2d(0.2, -0.2, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.25, makeDriveStep(new Pose2d(1.8, 1.5, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, makeDriveStep(new Pose2d(4.0, 0.0, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(2.0, makeDriveStep(new Pose2d(0.65, 0.0, new Rotation2d(Math.PI)))))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(makeDriveStep(new Pose2d(7.2, -0.14, new Rotation2d(Math.PI))));
            
        // Amp side speaker - shoot preload, WAIT 5 SECONDS, then dine and dash
        startingPositions[12] = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
        procedures[12] = new AutonomousProcedure("Amp Side Preload -> Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.5, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(7.0, (prevState) -> StepStatus.Running))
            .wait(makeDriveStep(new Pose2d(7.2, -0.14, new Rotation2d(Math.PI))));
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
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");            
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            System.out.println("BAD SWITCH VALUE!!!!!!!!!!!!!!");
            return new AutonomousProcedure("Unit");
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
