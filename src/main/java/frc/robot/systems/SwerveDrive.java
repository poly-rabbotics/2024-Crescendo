// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.io.IOException;

import java.nio.file.Path;

import java.util.function.BiFunction;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.SmartPrintable;

import frc.robot.subsystems.Angle;
import frc.robot.subsystems.StatusedTimer;
import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.PathPosition;

/*
 * Manages the swerve drive train.
 */
public class SwerveDrive extends SmartPrintable {
    // CAN IDs of drive motors and encoders.
    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1,   2,   3,   4  };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 5,   6,   7,   8  };
    private static final int MODULE_CANCODER_CAN_IDS[] = { 9,   10,  11,  12 };
    
    // Side length of the drive. Assumes a square drive.
    private static final double CHASSIS_SIDE_LENGTH = 0.58;

    // Offsets of each absolute encoder (in this case CANCoders) from zero, where we take an angle
    // of zero to be forward. By adding an offset to the corosponding encoder's reported angle, we
    // should be given the actual angle of the swerve module's wheel.
    private static final Angle MODULE_CANCODER_OFFSETS[] = {
        new Angle().setDegrees(75.498046875), 
        new Angle().setDegrees(-222.802734375), 
        new Angle().setDegrees(-73.388671875), 
        new Angle().setDegrees(57.216796875)  
    };

    // Positions for each wheel in "Rock Mode".
    private static final Angle MODULE_ROCK_MODE_POSITIONS[] = { 
        new Angle().setRadians( -Angle.TAU / 8  ), 
        new Angle().setRadians(  Angle.TAU / 8  ), 
        new Angle().setRadians( -Angle.TAU / 8  ), 
        new Angle().setRadians(  Angle.TAU / 8  ) 
    };

    // The physical positions of the drive's modules from the drive's center, in meters. The drive's
    // center need not be the robot's center.
    private static final Translation2d MODULE_PHYSICAL_POSITIONS[] = {
        new Translation2d( -CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
        new Translation2d( -CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
        new Translation2d(  CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
        new Translation2d(  CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
    };

    // PID parameter constants for following autonomous trajectories. These are used to follow a
    // setpoint given as a two dimensional point and a rotation.
    private static final double TRAJECTORY_STRAFE_X_PID_P = 0.4;
    private static final double TRAJECTORY_STRAFE_X_PID_I = 0.004;
    private static final double TRAJECTORY_STRAFE_X_PID_D = 0.035;

    private static final double TRAJECTORY_STRAFE_Y_PID_P = 0.6;
    private static final double TRAJECTORY_STRAFE_Y_PID_I = 0.004;
    private static final double TRAJECTORY_STRAFE_Y_PID_D = 0.035;

    private static final double TRAJECTORY_ROTATE_PID_P = 1.0;
    private static final double TRAJECTORY_ROTATE_PID_I = 0.0;
    private static final double TRAJECTORY_ROTATE_PID_D = 0.0;

    // Set angle constants. Use in teleop for "snap angles" available to the driver, or any other
    // controls function in teleop in which the driver may determine a desired angle, rather than
    // controlling the speed ot rotation itself.
    private static final double SET_ANGLE_PID_P = 1.0;
    private static final double SET_ANGLE_PID_I = 0.0;
    private static final double SET_ANGLE_PID_D = 0.0;

    // Singleton instance.
    private static final SwerveDrive instance = new SwerveDrive();

    // Module objects. Stores the module objects themselves which allow us to interface with them.
    private final SwerveModule modules[] = new SwerveModule[MODULE_MOVEMENT_CAN_IDS.length];

    // Positions of the modules, are used for things like odometry.
    private final SwerveModulePosition positions[] = new SwerveModulePosition[MODULE_MOVEMENT_CAN_IDS.length];

    // Kinematics takes our desired drive state and produces states for each module.
    private final SwerveDriveKinematics kinematics;

    // Odometry keeps track of our position on the field.
    private final SwerveDriveOdometry odometry;

    // PID controllers for autonomous trajectory following. Despite being an autonomous ability,
    // these controllers could be used in teleop, as the drive mode that uses them is always
    // available. Keep this in mind if auto-alignment to a target is desired and we have an accurate
    // source of odometry data.
    private final PIDController trajectoryStrafeXController
        = new PIDController(TRAJECTORY_STRAFE_X_PID_P, TRAJECTORY_STRAFE_X_PID_I, TRAJECTORY_STRAFE_X_PID_D);
    private final PIDController trajectoryStrafeYController
        = new PIDController(TRAJECTORY_STRAFE_Y_PID_P, TRAJECTORY_STRAFE_Y_PID_I, TRAJECTORY_STRAFE_Y_PID_D);
    private final PIDController trajectoryRotateController
        = new PIDController(TRAJECTORY_ROTATE_PID_P, TRAJECTORY_ROTATE_PID_I, TRAJECTORY_ROTATE_PID_D);

    // Controller for set angles, may be used on command by the driver while retaining full 
    // translation control. This controller is still available to the whole class, and so may be 
    // employed elsewhere.
    private final PIDController setAngleController
        = new PIDController(SET_ANGLE_PID_P, SET_ANGLE_PID_I, SET_ANGLE_PID_D);

    // These are for using pathweaver, which uses a time to determine the objective point.
    private StatusedTimer trajectoryTimer = new StatusedTimer();
    private Trajectory autonomousTrajectory = new Trajectory();

    // For sidewalk paver, which simply sets the current point through an autonomous procedure.
    private PathPosition setPathPosition = null;
    
    // Control curves for teleop run methods. The translation curve is two dimensional since both 
    // axes of the controller should be taken in context, rotation is simply one dimensional. The
    // intactive curves of either are for tracking what curve to return to when setting temporary
    // curves, which are used for hold-button functions rather than toggle-button ones. This allows
    // a curve to only be active when holdling a button, for example, a button to reduce speed may
    // be a hold-button.
    private BiFunction<Double, Double, Double> translationCurve = Controls::defaultCurveTwoDimensional;
    private BiFunction<Double, Double, Double> inactiveTransationCurve = null;
    private Function<Double, Double> rotationCurve = Controls::defaultCurve;
    private Function<Double, Double> inactiveRotationCurve = null;

    // Drive mode fields. Inactive mode works the same as an inactive curve, and tracks the drive
    // mode to return to when setting temporary modes. The display mode is used only for display,
    // and is useful when displaying aspects of the drive on something like LED lights.
    private SwerveMode mode = SwerveMode.HEADLESS;
    private SwerveMode inactiveMode = null;
    private SwerveMode displayMode = SwerveMode.HEADLESS;

    // Drive state objects. Module states represent the state of each module as independent items,
    // they are used as input to each modules and allow us to control them within code. The two 
    // chassis state objects are for the desired state of the drive and the state we measure via the
    // encoders on each modules. Chassis states may be displayed with the "swerve" widget in 
    // AdvantageScope, which is very helpful when debugging drive related issues.
    private SwerveModuleState[] moduleStates = { new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
    private ChassisSpeeds chassisSpeedsOutput = new ChassisSpeeds();
    private ChassisSpeeds chassisSpeedsCalculated = new ChassisSpeeds();

    // Target angle of the drive. This can be set from elsewhere in code via a setter method and can
    // allow the driver to control the angle of the drive, rather than rotation speed.
    // NOTE: See set angle controller and relevant constants above.
    private Angle setAngle = new Angle().setRadians(0.0);

    // Translation speeds along the X and Y axes as well as rotation speed. These are set by drive
    // modes and can be accessed elsewhere in code. They are used to form ChassisSpeeds objects.
    private double translationSpeedX = 0.0;
    private double translationSpeedY = 0.0;
    private double rotationSpeed = 0.0;
    private double ampChargeSpeed = 0.0;

    // Coefficiants for modifying trajectory following. Allows for the scaling and inverting of
    // either axis.
    private double trajectoryCoefficiantX = 1.0;
    private double trajectoryCoefficiantY = 1.0;

    private SwerveDrive() {
        super();
        
        // Create swerve modules using device CAN IDs, encoder offsets, and physical positions.
        for (int i = 0; i < MODULE_MOVEMENT_CAN_IDS.length; i++) {
            modules[i] = new SwerveModule(
                MODULE_MOVEMENT_CAN_IDS[i], 
                MODULE_ROTATION_CAN_IDS[i], 
                MODULE_CANCODER_CAN_IDS[i], 
                MODULE_CANCODER_OFFSETS[i], 
                MODULE_PHYSICAL_POSITIONS[i]
            );
        }

        // Populate the modules positions for later.
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        // Create the kinematics object. From now on all interfacin with kinematics must be done
        // with modules in the same order. We've already set up all our module related objects in
        // arrays of length four in the same order, which goes front right, back right, back left,
        // front left.
        kinematics = new SwerveDriveKinematics(
            MODULE_PHYSICAL_POSITIONS[0],
            MODULE_PHYSICAL_POSITIONS[1],
            MODULE_PHYSICAL_POSITIONS[2],
            MODULE_PHYSICAL_POSITIONS[3]
        );

        // Odometry will be interfaced with in the same order as kinematics are now. It also
        // requires the angle of our gyro and the positions we recorder earlier.
        odometry = new SwerveDriveOdometry(
            kinematics, 
            new Rotation2d(Pigeon.getYaw().radians()), 
            positions
        );

        // Enable continuous input and set tolerance on the rotation controllers. Continuous input
        // between zero and tau will make the controller recognize that it works on a circle, and
        // that a position at zero or any multiple of tau are the same position.
        trajectoryRotateController.enableContinuousInput(0.0, Angle.TAU);
        trajectoryRotateController.setTolerance(0.1);
        setAngleController.enableContinuousInput(0.0, Angle.TAU);
        setAngleController.setTolerance(0.1);
    }

    /**
     * Sets the current mode of the swerve drive. This may change the way run methods work.
     * @param mode The mode in which to operate.
     */
    public static void setMode(SwerveMode mode) {
        instance.mode = mode;
    }

    /**
     * Get the current swerve mode.
     */
    public static SwerveMode getMode() {
        return instance.mode;
    }

    /**
     * Gets the current display mode of the drive. The display mode is the
     * either the current set mode or a temporary set mode but is not cleared
     * at the end of a run loop, rather it is cleared once the next loop 
     * finishes, making it ideal for display purposes (e.i. LED lights).
     */
    public static SwerveMode getDisplayMode() {
        return instance.displayMode;
    }

    /**
     * May be called before running the swerve drive to temporarily set a mode
     * for that call of a run method.
     * 
     * Each time a run method completes it will change the mode back to what it
     * was when this method was called.
     * 
     * If this method is called more than once before running then the latest
     * given temporary mode will be used, the inactive mode to be reset to after
     * running will not be effected
     */
    public static void tempMode(SwerveMode mode) {
        if (instance.inactiveMode != null) {
            instance.mode = mode;
            return;
        }

        instance.inactiveMode = instance.mode;
        instance.mode = mode;
    }

    /**
     * Exactly like `tempMode` but predicates the temporary mode on the given 
     * boolean condition.
     */
    public static void conditionalTempMode(SwerveMode mode, boolean condition) {
        if (!condition) {
            return;
        }

        tempMode(mode);
    }

    /**
     * Gets the translation speed along the X axis. In headless mode this gives
     * a speed along the field's x-axis, otheriwse it is the forward speed of 
     * the robot.
     */
    public static double getTranslationSpeedX() {
        return instance.translationSpeedX;
    }

    /**
     * Gets the translation speed along the Y axis. In headless mode this gives
     * a speed along the field's y-axis, otheriwse it is the sideways speed of 
     * the robot.
     */
    public static double getTranslationSpeedY() {
        return instance.translationSpeedY;
    }

    /**
     * Gets the rotation speed of the robot.
     */
    public static double getRotationSpeed() {
        return instance.rotationSpeed;
    }

    /**
     * Sets the curve function for directional inputs (translations).
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void setTranslationCurve(BiFunction<Double, Double, Double> curve) {
        instance.translationCurve = curve;
    }

    /**
     * Gets the current curve used for directional inputs.
     */
    public static BiFunction<Double, Double, Double> getTranslationCurve() {
        return instance.translationCurve;
    }

    /**
     * Temporarily sets the curve function for directional inputs (translations).
     * This action will atomatically be undone after calling a run method.
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void tempTranslationCurve(BiFunction<Double, Double, Double> curve) {
        if (instance.inactiveTransationCurve != null) {
            instance.translationCurve = curve;
            return;
        }

        instance.inactiveTransationCurve = instance.translationCurve;
        instance.translationCurve = curve;
    }

    /**
     * Exactly like `tempDirectionalCurve` but predicated on a boolean condition.
     */
    public static void conditionalTempTranslationCurve(
        BiFunction<Double, Double, Double> curve, 
        boolean condition
    ) {
        if (!condition) {
            return;
        }

        tempTranslationCurve(curve);
    }

    /**
     * Sets the curve function for turn inputs.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void setRotationCurve(Function<Double, Double> curve) {
        instance.rotationCurve = curve;
    }

    /**
     * Gets the Function currently used for turning.
     */
    public static Function<Double, Double> getRotationCurve() {
        return instance.rotationCurve;
    }

    /**
     * Temporarily sets the curve function for turn inputs. Undone after running.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void tempRotationCurve(Function<Double, Double> curve) {
        if (instance.inactiveRotationCurve != null) {
            instance.rotationCurve = curve;
            return;
        }

        instance.inactiveRotationCurve = instance.rotationCurve;
        instance.rotationCurve = curve;
    }

    /**
     * Exactly like `tempTurnCurve` but predicated on a boolean condition.
     */
    public static void conditionalTempRotationCurve(
        Function<Double, Double> curve, 
        boolean condition
    ) {
        if (!condition) {
            return;
        }

        tempRotationCurve(curve);
    }

    /**
     * Runs with a static speed of zero for all inputs, meant to be used for
     * trajectory following in autonomous.
     */
    public static void run() {
        runUncurved(0.0, 0.0, 0.0);
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. This will reset
     * temporary modes on completion.
     * @param translationX The X axis of the directional control, between 1 and -1
     * @param translationY The Y axis of the directional control, between 1 and -1.
     * @param speed The speed scalar for the drive.
     * @param rotation A value between 1 and -1 that determines the turning speed.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(
        double translationX,
        double translationY,
        double speed,
        double rotation
    ) {
        speed = Math.abs(translationX) <= 0.05 && Math.abs(translationY) <= 0.05 ? 0.0 : speed;

        // angle is in radians as per Java's trig methods.
        var angle = Math.atan2(translationY, translationX);
        translationX = Math.cos(angle) * speed;

        translationY = Math.sin(angle) * speed;
        run(translationX, translationY, rotation);
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. Derives speed
     * from directional inputs. This will reset temporary modes on completion.
     * @param translationX The X axis of the directional control, between 1 and -1
     * @param translationY The Y axis of the directional control, between 1 and -1.
     * @param rotation A value between 1 and -1 that determines the turning speed.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(double translationX, double translationY, double rotation) {
        var x = instance.translationCurve.apply(translationX, translationY);
        var y = instance.translationCurve.apply(translationY, translationX);
        rotation = instance.rotationCurve.apply(rotation);
        runUncurved(x, y, rotation);
    }
    
    /**
     * Run the swerve drive exactly by the arguments passed, no curving will 
     * occure on the given inputs, nor will they be deadbanded. This will reset
     * temporary modes on completion.
     * @param translationX Speed along the x axis. -1.0 - 1.0
     * @param translationY Speed along the y axis. -1.0 - 1.0
     * @param rotation Rate of turn. -1.0 - 1.0
     */
    public static void runUncurved(double translationX, double translationY, double rotation) {
        instance.translationSpeedX = translationX;
        instance.translationSpeedY = translationY;
        instance.rotationSpeed = rotation;

        SwerveModuleState[] moduleStates = new SwerveModuleState[instance.modules.length];
        boolean holdPos = false;

        switch (instance.mode) {
            case HEADLESS: {
                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        instance.rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case RELATIVE: {
                moduleStates = instance.kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        instance.rotationSpeed, 
                        new Rotation2d(0.0)
                    )
                ); 
                break;
            }

            case ROCK: {
                assert moduleStates.length == instance.modules.length;
                holdPos = true;

                for (int i = 0; i < instance.modules.length; i++) {
                    moduleStates[i] = new SwerveModuleState(
                        0.0,
                        new Rotation2d(
                            MODULE_ROCK_MODE_POSITIONS[i].radians()
                        )
                    );
                }

                break;
            }

            case SET_ANGLE: {
                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        -instance.setAngleController.calculate(
                            Pigeon.getYaw().radians(),
                            instance.setAngle.radians()
                        ), 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case AIMBOT_ROTATION: {
                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        Aimbot.calculateTurn(), 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case AIMBOT: {
                moduleStates = instance.kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                        instance.translationSpeedX,
                        Aimbot.isCentered()
                            ? Aimbot.calculateMovement()
                            : 0.0,
                        Aimbot.calculateTurn()
                    )
                );
                break;
            }

            case TRAJECTORY_FOLLOW: {
                if (instance.autonomousTrajectory == null || instance.autonomousTrajectory.getTotalTimeSeconds() + 0.35 < instance.trajectoryTimer.get()) {
                    instance.translationSpeedX = 0.0;
                    instance.translationSpeedY = 0.0;
                    instance.rotationSpeed = 0.0;

                    moduleStates = instance.kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(0.0, 0.0, 0.0)
                    );

                    break;
                }

                State state = instance.autonomousTrajectory.sample(instance.trajectoryTimer.get());
                Pose2d setPosition = state.poseMeters;
                Pose2d position = instance.odometry.getPoseMeters();

                instance.translationSpeedX
                    = instance.trajectoryStrafeXController.calculate(
                        position.getY(), 
                        setPosition.getY()
                    );
                instance.translationSpeedY
                    = instance.trajectoryStrafeYController.calculate(
                        position.getX(),
                        -setPosition.getX()
                    );
                instance.rotationSpeed = -instance.trajectoryRotateController.calculate(
                    position.getRotation().getRadians(),
                    setPosition.getRotation().getRadians()
                );

                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        instance.rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );

                break;
            }

            case SIDEWALK_WALK: {
                if (instance.setPathPosition == null) {
                    // Break early if we have no path.

                    moduleStates = instance.kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, 
                            new Rotation2d(Pigeon.getYaw().radians())
                        )
                    );

                    break;
                }

                Pose2d setPosition = instance.setPathPosition.pose;
                Pose2d position = instance.odometry.getPoseMeters();

                instance.translationSpeedX
                    = instance.trajectoryStrafeXController.calculate(position.getX(), setPosition.getX());
                instance.translationSpeedY
                    = instance.trajectoryStrafeYController.calculate(position.getY(), setPosition.getY());
                instance.rotationSpeed = -instance.trajectoryRotateController.calculate(
                    position.getRotation().getRadians(),
                    setPosition.getRotation().getRadians()
                );

                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        instance.rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );

                break;
            }

            case AMP_LINE_UP: {
                instance.translationSpeedX = Math.abs(instance.ampChargeSpeed) > 0.1 
                    ? 0.0
                    : Aimbot.ampLineUpX() * -getTrajectoryCoefficiantY();
                instance.translationSpeedY = instance.ampChargeSpeed * getTrajectoryCoefficiantY();
                instance.setAngle = new Angle().setDegrees(-90.0 * getTrajectoryCoefficiantY());

                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        instance.translationSpeedX,
                        instance.translationSpeedY,
                        -instance.setAngleController.calculate(
                            Pigeon.getYaw().radians(),
                            instance.setAngle.radians()
                        ), 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
            }

            // This branch should never be reached as the enum used should never
            // have more than the above possible values.
            default: assert false;
        }

        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(moduleStates[i]);
            instance.modules[i].setRockMode(holdPos);
            instance.modules[i].run();
        }

        // Record module states

        instance.moduleStates = moduleStates;
        
        // Reset temporary states
        
        instance.displayMode = instance.mode;
        
        if (instance.inactiveMode != null) {
            instance.mode = instance.inactiveMode;
            instance.inactiveMode = null;
        }
        
        if (instance.inactiveTransationCurve != null) {
            instance.translationCurve = instance.inactiveTransationCurve;
            instance.inactiveTransationCurve = null;
        }

        if (instance.inactiveRotationCurve != null) {
            instance.rotationCurve = instance.inactiveRotationCurve;
            instance.inactiveRotationCurve = null;
        }
    }

    /**
     * Record states for logging and displays like AdvantageKit.
     */
    public static void recordStates() {
        SwerveModuleState[] measuredState = new SwerveModuleState[instance.modules.length];

        for (int i = 0; i < instance.modules.length; i++) {
            measuredState[i] = instance.modules[i].getActualState();
        }

        Logger.recordOutput("Swerve Module States Measured", measuredState);
        Logger.recordOutput("Swerve Module States", instance.moduleStates);

        if (instance.setPathPosition != null) {
            Logger.recordOutput("Swerve Drive Target Pose", instance.setPathPosition.pose);
        } else {
            Logger.recordOutput("Swerve Drive Target Pose", new Pose2d());
        }

        Logger.recordOutput("Swerve Odometry", getOdometryPose());
        Logger.recordOutput("Swerve Applied Current (amps)", getAppliedCurrent());
        Logger.recordOutput("Swerve Average Motor Tempurature (celsius)", getAverageMotorTemp());

        Logger.recordOutput("Swerve Speed Rotation", getRotationSpeed());
        Logger.recordOutput("Swerve Speed Translation X", getTranslationSpeedX());
        Logger.recordOutput("Swerve Speed Translation Y", getTranslationSpeedY());

        Logger.recordOutput("Swerve Drive Mode", getMode());
        Logger.recordOutput("Swerve Display Mode", getDisplayMode());

        Logger.recordOutput("Swerve Chassis Speeds Output Vx", instance.chassisSpeedsOutput.vxMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Output Vy", instance.chassisSpeedsOutput.vyMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Output Tau", instance.chassisSpeedsOutput.omegaRadiansPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Calculated Vx", instance.chassisSpeedsCalculated.vxMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Calculated Vy", instance.chassisSpeedsCalculated.vyMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Calculated Tau", instance.chassisSpeedsCalculated.omegaRadiansPerSecond);

        Logger.recordOutput("Swerve Trajectory Coefficiant X", instance.trajectoryCoefficiantX);
        Logger.recordOutput("Swerve Trajectory Coefficiant Y", instance.trajectoryCoefficiantY);
    }

    /**
     * Updates module positions and drive odometry.
     */
    public static void updateOdometry() {
        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].updatePosition();
            instance.positions[i] = instance.modules[i].getPosition();
        }

        instance.chassisSpeedsOutput = instance.kinematics.toChassisSpeeds(
            instance.modules[0].getDesiredState(),
            instance.modules[1].getDesiredState(),
            instance.modules[2].getDesiredState(),
            instance.modules[3].getDesiredState()
        );

        instance.chassisSpeedsCalculated = instance.kinematics.toChassisSpeeds(
            instance.modules[0].getActualState(),
            instance.modules[1].getActualState(),
            instance.modules[2].getActualState(),
            instance.modules[3].getActualState()
        );

        instance.odometry.update(new Rotation2d(Pigeon.getYaw().radians()), instance.positions);
    }
    
    /**
     * Gets current odometry position.
     */
    public static Pose2d getOdometryPose() {
        return instance.odometry.getPoseMeters();
    }

    /**
     * Zeros position entirely, assuming the robot is facing forward, then set
     * the odometry position to the given X and Y components.
     */
    public static void setOdometry(Pose2d pose) {
        //zeroPositions();

        instance.odometry.resetPosition(
            new Rotation2d(Pigeon.getYaw().radians()),
            instance.positions,
            pose
        );
    }
    
    /**
     * Zeros all movement encoder positions.
     */
    public static void zeroPositions() {
        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].zeroPositions();
            instance.positions[i] = instance.modules[i].getPosition();
        }

        instance.odometry.resetPosition(
            new Rotation2d(Pigeon.getYaw().radians()), 
            instance.positions, 
            new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        );
    }

    public static void setAmpChargeSpeed(double speed) {
        instance.ampChargeSpeed = speed;
    }

    /**
     * Gets the average tempurature of all motors on the drive in celsius.
     */
    public static double getAverageMotorTemp() {
        double tempSum = 0.0;

        for (SwerveModule module : instance.modules) {
        	tempSum += module.getRotationMotorTemp();
            tempSum += module.getMovementMotorTemp();
        }

        return tempSum / (instance.modules.length * 2.0);
    }

    /**
     * Gets the sum of all applied currents in amps of all motors on the drive.
     */
    public static double getAppliedCurrent() {
        double current = 0.0;

        for (SwerveModule module : instance.modules) {
        	current += module.getAppliedCurrent();
        }

        return current;
    }

    /**
     * Gets the average percent usage of each module's motor controller 
     * current pull.
     */
    public static double getAveragePercentRatedCurrent() {
        double percentSum = 0.0;

        for (SwerveModule module : instance.modules) {
        	percentSum += module.getPercentRatedCurrent();
        }

        return percentSum / (double)instance.modules.length;
    }

    /**
     * Loads a PathWeaver JSON file to be used in the TRAJECTORY_FOLLOW drive
     * mode. This function will set the odometry position to the beginning of
     * trajectory.
     */
    public static void grabPathweaverFile(String filePath) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + filePath);
            instance.autonomousTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            
            Pose2d initialPosition = instance.autonomousTrajectory.getInitialPose();
            initialPosition = new Pose2d(
                -initialPosition.getX(),
                initialPosition.getY(),
                initialPosition.getRotation()
            );

            instance.odometry.resetPosition(
                new Rotation2d(Pigeon.getYaw().radians()),
                instance.positions,
                initialPosition
            );
        } catch (IOException e) {
            DriverStation.reportError(
                "Could not open trajectory at '" + filePath + "'",
                e.getStackTrace()
            );
        }
    }

    /**
     * Starts the trajectory timer at zero time elapsed.
     */
    public static void startTrajectoryTimer() {
        instance.trajectoryTimer.reset();
        instance.trajectoryTimer.start();
    }

    /**
     * Sets the set angle for `SET_ANGLE` drive mode.
     */
    public static void setTargetAngle(Angle angle) {
        instance.setAngle = angle;
    }

    /**
     * Sets the path position for the `SIDEWALK_WALK` mode.
     */
    public static void setTargetPathPosition(PathPosition setPathPosition) {
        PathPosition pose = new PathPosition(
            new Pose2d(
                setPathPosition.pose.getX() * instance.trajectoryCoefficiantX, 
                setPathPosition.pose.getY() * instance.trajectoryCoefficiantY, 
                instance.trajectoryCoefficiantY < 0.0
                    ? new Rotation2d(Angle.TAU - setPathPosition.pose.getRotation().getRadians())
                    : setPathPosition.pose.getRotation()
            ),
            setPathPosition.timeSeconds
        );

        

        instance.setPathPosition = pose;
    }

    public static boolean withinPositionTolerance() {
        final double TOLERANCE_X = 0.3;
        final double TOLERANCE_Y = 0.3;
        final double TOLERANCE_THETA = 0.1;

        // makes sure angle are within [0, tau)
        double measuredAngle = (getOdometryPose().getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;
        double setAngle = (instance.setPathPosition.pose.getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;

        return Math.abs(getOdometryPose().getX() - instance.setPathPosition.pose.getX()) < TOLERANCE_X
            && Math.abs(getOdometryPose().getY() - instance.setPathPosition.pose.getY()) < TOLERANCE_Y
            && Math.abs(measuredAngle - setAngle) < TOLERANCE_THETA;
    }

    public static void setTrajectoryCoefficiants(double x, double y) {
        instance.trajectoryCoefficiantX = x;
        instance.trajectoryCoefficiantY = y;
    }

    public static double getTrajectoryCoefficiantX() {
        return instance.trajectoryCoefficiantX;
    }

    public static double getTrajectoryCoefficiantY() {
        return instance.trajectoryCoefficiantY;
    }

    public static void setPropulsionLimit(double limit) {
        for (SwerveModule module : instance.modules) {
            module.setPropulsionLimit(limit);
        }
    }

    /**
     * Print data to smart dashboard.
     */
    @Override
    public void print() {
        SmartDashboard.putString("Swerve Drive Mode", getMode().toString());
        SmartDashboard.putString("Swerve Drive Odometry", 
            "(" + ((double)(long)(odometry.getPoseMeters().getX() * 100)) / 100 + ", "
            + ((double)(long)(odometry.getPoseMeters().getY() * 100)) / 100 + ") "
            + ((double)(long)(odometry.getPoseMeters().getRotation().getDegrees() * 100)) / 100 + " degrees"
        );
        SmartDashboard.putNumber("Swerve Drive Average Motor Tempurature (Celsius)", getAverageMotorTemp());
        SmartDashboard.putNumber("Swerve Drive Total Current Pull (Amps)", getAppliedCurrent());
        SmartDashboard.putNumber("Swerve Drive Translation Speed X", translationSpeedX);
        SmartDashboard.putNumber("Swerve Drive Translation Speed Y", translationSpeedY);
        SmartDashboard.putNumber("Swerve Drive Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("Swerve Drive Chassis Speeds Output Vx", chassisSpeedsOutput.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve Drive Chassis Speeds Output Vy", chassisSpeedsOutput.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve Drive Chassis Speeds Output Tau", chassisSpeedsOutput.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Swerve Drive Chassis Speeds Calculated Vx", chassisSpeedsCalculated.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve Drive Chassis Speeds Calculated Vy", chassisSpeedsCalculated.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve Drive Chassis Speeds Calculated Tau", chassisSpeedsCalculated.omegaRadiansPerSecond);

        SmartDashboard.putString("Swerve Drive Trajectory", autonomousTrajectory.toString());
        SmartDashboard.putNumber("Swerve Trajectory Timer Time", trajectoryTimer.get());

        try {
            var state = autonomousTrajectory.sample(trajectoryTimer.get());

            SmartDashboard.putString("Swerve Drive Trajectory Current State", state.toString());
            SmartDashboard.putNumber("Swerve Drive Trajectory Position X", state.poseMeters.getX());
            SmartDashboard.putNumber("Swerve Drive Trajectory Position Y", state.poseMeters.getY());
            SmartDashboard.putNumber("Swerve Drive Trajectory Rotation Degrees", state.poseMeters.getRotation().getDegrees());
            SmartDashboard.putNumber("Swerve Drive Trajectory Time", state.timeSeconds);
            SmartDashboard.putBoolean("Swerve Drive Trajectory Time Finished", instance.autonomousTrajectory.getTotalTimeSeconds() < instance.trajectoryTimer.get());
        } catch (Exception e) {
            SmartDashboard.putString("Swerve Drive Trajectory Current State", "No State");
	    }

        if (setPathPosition != null) {
            SmartDashboard.putNumber("Swerve Drive Sidewalk Paver Pose X", setPathPosition.pose.getX());
            SmartDashboard.putNumber("Swerve Drive Sidewalk Paver Pose Y", setPathPosition.pose.getY());
            SmartDashboard.putNumber("Swerve Drive Sidewalk Paver Pose Rotation Degrees", setPathPosition.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Swerve Drive Sidewalk Paver Pose Discrete Time", setPathPosition.timeSeconds);
        }
    }
}
