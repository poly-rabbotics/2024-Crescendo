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
        startingPositions[0] = new Pose2d(0.0, 0.0, new Rotation2d(0));
        procedures[0] = new AutonomousProcedure("Unit Procedure");

        // MODE 1 - SPEAKER SCORE ONLY
        startingPositions[1] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[1] = new AutonomousProcedure("Speaker 1 Note")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait((prevState) -> Hands.shooter.set(ShooterState.RUNNING))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

        /* MODE 2 // DRIVE TO AMP AND SCORE */
        /* startingPositions[2] = new Pose2d(0.0, 0.0, new Rotation2d(0));
        procedures[2] = new AutonomousProcedure("")
            .wait((prevState) -> StepStatus.Running) //SWERVE DRIVING PART
            .wait(AutonomousProcedure.timeoutAt(2, (prevState) -> Hands.pivot.set(Setpoint.AMP_SCORING)))
            .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0.5)))
            .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0.0)))
            .wait(AutonomousProcedure.timeoutAt(0.3, (prevState) -> StepStatus.Running))
            .wait(AutonomousProcedure.timeoutAt(1, (prevState) -> Hands.linearActuator.setPosition(0)))
            .wait(AutonomousProcedure.timeoutAt(2, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))); */

        // START CENTER SPEAKER - 2 NOTE
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
        
        // START SOURCE SIDE - 2 NOTE
        startingPositions[4] = new Pose2d(0.0, 0.0, new Rotation2d(2.366));
        procedures[4] = new AutonomousProcedure("Speaker Score -> Leave -> Intake -> Move Back -> Speaker Score")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait((prevState) -> {
                var pose = new Pose2d(1.75, -0.15, new Rotation2d(0.0));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait(AutonomousProcedure.timeoutAt(1.75, (prevState) -> {
                var pose = new Pose2d(2.25, -0.15, new Rotation2d(0.0));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            }))
            .wait((prevState) -> {
                var pose = new Pose2d(1.5, -0.15, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait((prevState) -> {
                var pose = new Pose2d(0.0, 0.0, new Rotation2d(2.366));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            })
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE));

        // START SOURCE SIDE
        startingPositions[5] = new Pose2d(0.0, 0.0, new Rotation2d(2.237));
        procedures[5] = new AutonomousProcedure("Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> {
                var pose = new Pose2d(0.0, -2.25, new Rotation2d(2.237));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }

                return StepStatus.Running;
            }))
            .wait((prevState) -> {
                var pose = new Pose2d(7.2, -2.25, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }

                return StepStatus.Running;
            });
        
        // START AMP SIDE
        startingPositions[6] = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
        procedures[6] = new AutonomousProcedure("Amp 2 Note -> Dine and Dash")
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> {
                var pose = new Pose2d(1.0, 0.4, new Rotation2d(0.0));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }

                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(2.25, (prevState) -> {
                var pose = new Pose2d(3.5, 0.4, new Rotation2d(0.0));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }

                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> {
                var pose = new Pose2d(0.0, 0.0, new Rotation2d(-2.237));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }

                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 2.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.loader.fire()))
            .wait((prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE))
            .wait((prevState) -> Hands.shooter.set(ShooterState.IDLE))
            .wait((prevState) -> {
                var pose = new Pose2d(7.2, -0.14, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }

                return StepStatus.Running;
            });
        
        // START CENTER SPEAKER
        startingPositions[7] = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI));
        procedures[7] = new AutonomousProcedure("Three Note")
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt( 1.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> {
                var pose = new Pose2d(2.0, 1.85, new Rotation2d(0.65));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> {
                var pose = new Pose2d(0.2, -0.2, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.pivot.set(Setpoint.GROUND_INTAKE)))
            .wait(AutonomousProcedure.timeoutAt(0.01, (prevState) -> Hands.shooter.set(ShooterState.IDLE)))
            .wait(AutonomousProcedure.timeoutAt(1.25, (prevState) -> {
                var pose = new Pose2d(1.8, 1.5, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> {
                var pose = new Pose2d(4.0, 0.0, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(2.0, (prevState) -> {
                var pose = new Pose2d(0.65, 0.0, new Rotation2d(Math.PI));
                SwerveDrive.setTargetPathPosition(new PathPosition(pose, 0.0));

                if (SwerveDrive.withinPositionTolerance()) {
                    return StepStatus.Done;
                }
                
                return StepStatus.Running;
            }))
            .wait(AutonomousProcedure.timeoutAt(1.0, (prevState) -> Hands.pivot.set(Setpoint.STATIC_SHOOTING)))
            .wait(AutonomousProcedure.timeoutAt(1.5, (prevState) -> Hands.shooter.set(ShooterState.RUNNING)))
            .wait(AutonomousProcedure.timeoutAt(0.65, (prevState) -> Hands.loader.fire()))
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
