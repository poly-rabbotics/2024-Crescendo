package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.PathPosition;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.Hands.Setpoint;
import frc.robot.systems.Hands.ShooterState;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 3;
    private static AutonomousManager instance = new AutonomousManager();
    private AutonomousProcedure[] procedures = new AutonomousProcedure[1 << (AUTO_SWITCH_COUNT - 1)];
    private Pose2d[] startingPositions = new Pose2d[1 << (AUTO_SWITCH_COUNT - 1)];

    /*
     * Assume autos start width the amp in the positive x direction.
     */

    private AutonomousManager() {
        procedures[0] = new AutonomousProcedure("Unit Procedure");

        /* MODE 1 // SPEAKER SCORE ONLY */
        procedures[1] = new AutonomousProcedure("Speaker Score")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait((prevState) -> Hands.shooter.set(ShooterState.RUNNING))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

        /* MODE 2 // DRIVE TO AMP AND SCORE */
        procedures[2] = new AutonomousProcedure("")
            .wait((prevState) -> StepStatus.Running) //SWERVE DRIVING PART
            .wait(AutonomousProcedure.timeoutAt(2, (prevState) -> Hands.pivot.set(Setpoint.AMP_SCORING)))
            .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0.5)))
            .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0.0)))
            .wait(AutonomousProcedure.timeoutAt(0.3, (prevState) -> StepStatus.Running))
            .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0)))
            .wait(AutonomousProcedure.timeoutAt(2, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)));

        startingPositions[3] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[3] = new AutonomousProcedure("Speaker Score -> Leave -> Intake -> Move Back -> Speaker Score")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait((prevState) -> {
                var pose = new Pose2d(1.25, 0.0, new Rotation2d(0.0));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait((prevState) -> {
                var pose = new Pose2d(2.25, 0.0, new Rotation2d(0.0));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait((prevState) -> {
                var pose = new Pose2d(1.25, 0.0, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait((prevState) -> {
                var pose = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));
        
            // START AMP SIDE
            startingPositions[4] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI + Math.PI / 6));
            procedures[4] = new AutonomousProcedure("Speaker Score -> Leave -> Intake -> Move Back -> Speaker Score")
                .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
                .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
                .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
                .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
                .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
                .wait((prevState) -> {
                    var pose = new Pose2d(1.75, 0.75, new Rotation2d(0.0));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait((prevState) -> {
                    var pose = new Pose2d(2.75, 0.75, new Rotation2d(0.0));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait((prevState) -> {
                    var pose = new Pose2d(1.75, 0.75, new Rotation2d(Math.PI + Math.PI / 6));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait((prevState) -> {
                    var pose = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI + Math.PI / 6));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
                .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
                .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
                .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
                .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

            // START STAGE SIDE
            startingPositions[5] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI - Math.PI / 6));
            procedures[5] = new AutonomousProcedure("Speaker Score -> Leave -> Intake -> Move Back -> Speaker Score")
                .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
                .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
                .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
                .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
                .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
                .wait((prevState) -> {
                    var pose = new Pose2d(1.75, -0.75, new Rotation2d(0.0));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait((prevState) -> {
                    var pose = new Pose2d(2.75, -0.75, new Rotation2d(0.0));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait((prevState) -> {
                    var pose = new Pose2d(1.75, -0.75, new Rotation2d(Math.PI - Math.PI / 6));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait((prevState) -> {
                    var pose = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI - Math.PI / 6));
                    SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));
    
                    if (SwerveDrive.withinPositionTolerance()) {
                        return StepStatus.Done;
                    }
                    
                    return StepStatus.Running;
                })
                .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
                .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
                .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
                .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
                .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));
    }

    public static void reset() {
        instance = new AutonomousManager();
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

    /**
     * Gets an auto start by its index.
     */
    public static Pose2d getStartingPos(int mode) {
        return instance.startingPositions[mode];
    }

    /**
     * Gets an auto start by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static Pose2d getStartingPosSwitches(boolean...  switches) {
        int mode = 0;

        for (int i = 0; i < switches.length; i++) {
            mode += switches[i] ? (1 << i) : 0;
        }

        System.out.println("SWITCHES AT::::::::::::::::::::::::::::::::::::::::::::" + mode);
        return getStartingPos(mode);
    }
}
