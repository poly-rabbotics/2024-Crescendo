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

import frc.robot.subsystems.Angle;
import frc.robot.subsystems.StatusedTimer;
import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.PathPosition;

/*
 * Manages the swerve drive train.
 */
public class SwerveDrive {
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

    // Module objects. Stores the module objects themselves which allow us to interface with them.
    private static final SwerveModule modules[] = new SwerveModule[MODULE_MOVEMENT_CAN_IDS.length];

    // Positions of the modules, are used for things like odometry.
    private static final SwerveModulePosition positions[] = new SwerveModulePosition[MODULE_MOVEMENT_CAN_IDS.length];

    // Kinematics takes our desired drive state and produces states for each module.
    private static final SwerveDriveKinematics kinematics;

    // Odometry keeps track of our position on the field.
    private static final SwerveDriveOdometry odometry;

    // PID controllers for autonomous trajectory following. Despite being an autonomous ability,
    // these controllers could be used in teleop, as the drive mode that uses them is always
    // available. Keep this in mind if auto-alignment to a target is desired and we have an accurate
    // source of odometry data.
    private static final PIDController trajectoryStrafeXController
        = new PIDController(TRAJECTORY_STRAFE_X_PID_P, TRAJECTORY_STRAFE_X_PID_I, TRAJECTORY_STRAFE_X_PID_D);
    private static final PIDController trajectoryStrafeYController
        = new PIDController(TRAJECTORY_STRAFE_Y_PID_P, TRAJECTORY_STRAFE_Y_PID_I, TRAJECTORY_STRAFE_Y_PID_D);
    private static final PIDController trajectoryRotateController
        = new PIDController(TRAJECTORY_ROTATE_PID_P, TRAJECTORY_ROTATE_PID_I, TRAJECTORY_ROTATE_PID_D);

    // Controller for set angles, may be used on command by the driver while retaining full 
    // translation control. This controller is still available to the whole class, and so may be 
    // employed elsewhere.
    private static final PIDController setAngleController
        = new PIDController(SET_ANGLE_PID_P, SET_ANGLE_PID_I, SET_ANGLE_PID_D);

    // These are for using pathweaver, which uses a time to determine the objective point.
    private static StatusedTimer trajectoryTimer = new StatusedTimer();
    private static Trajectory autonomousTrajectory = new Trajectory();

    // For sidewalk paver, which simply sets the current point through an autonomous procedure.
    private static PathPosition setPathPosition = null;
    
    // Control curves for teleop run methods. The translation curve is two dimensional since both 
    // axes of the controller should be taken in context, rotation is simply one dimensional. The
    // intactive curves of either are for tracking what curve to return to when setting temporary
    // curves, which are used for hold-button functions rather than toggle-button ones. This allows
    // a curve to only be active when holdling a button, for example, a button to reduce speed may
    // be a hold-button.
    private static BiFunction<Double, Double, Double> translationCurve = Controls::defaultCurveTwoDimensional;
    private static BiFunction<Double, Double, Double> inactiveTransationCurve = null;
    private static Function<Double, Double> rotationCurve = Controls::defaultCurve;
    private static Function<Double, Double> inactiveRotationCurve = null;

    // Drive mode fields. Inactive mode works the same as an inactive curve, and tracks the drive
    // mode to return to when setting temporary modes. The display mode is used only for display,
    // and is useful when displaying aspects of the drive on something like LED lights.
    private static SwerveMode mode = SwerveMode.HEADLESS;
    private static SwerveMode inactiveMode = null;
    private static SwerveMode displayMode = SwerveMode.HEADLESS;

    // Drive state objects. Module states represent the state of each module as independent items,
    // they are used as input to each modules and allow us to control them within code. The two 
    // chassis state objects are for the desired state of the drive and the state we measure via the
    // encoders on each modules. Chassis states may be displayed with the "swerve" widget in 
    // AdvantageScope, which is very helpful when debugging drive related issues.
    private static SwerveModuleState[] moduleStates = { new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
    private static ChassisSpeeds chassisSpeedsOutput = new ChassisSpeeds();
    private static ChassisSpeeds chassisSpeedsCalculated = new ChassisSpeeds();

    // Target angle of the drive. This can be set from elsewhere in code via a setter method and can
    // allow the driver to control the angle of the drive, rather than rotation speed.
    // NOTE: See set angle controller and relevant constants above.
    private static Angle setAngle = new Angle().setRadians(0.0);

    // Translation speeds along the X and Y axes as well as rotation speed. These are set by drive
    // modes and can be accessed elsewhere in code. They are used to form ChassisSpeeds objects.
    private static double translationSpeedX = 0.0;
    private static double translationSpeedY = 0.0;
    private static double rotationSpeed = 0.0;
    private static double ampChargeSpeed = 0.0;

    // Coefficiants for modifying trajectory following. Allows for the scaling and inverting of
    // either axis.
    private static double trajectoryCoefficiantX = 1.0;
    private static double trajectoryCoefficiantY = 1.0;

    static {
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
        SwerveDrive.mode = mode;
    }

    /**
     * Get the current swerve mode.
     */
    public static SwerveMode getMode() {
        return mode;
    }

    /**
     * Gets the current display mode of the drive. The display mode is the
     * either the current set mode or a temporary set mode but is not cleared
     * at the end of a run loop, rather it is cleared once the next loop 
     * finishes, making it ideal for display purposes (e.i. LED lights).
     */
    public static SwerveMode getDisplayMode() {
        return displayMode;
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
        if (inactiveMode != null) {
            SwerveDrive.mode = mode;
            return;
        }

        inactiveMode = mode;
        SwerveDrive.mode = mode;
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
        return translationSpeedX;
    }

    /**
     * Gets the translation speed along the Y axis. In headless mode this gives
     * a speed along the field's y-axis, otheriwse it is the sideways speed of 
     * the robot.
     */
    public static double getTranslationSpeedY() {
        return translationSpeedY;
    }

    /**
     * Gets the rotation speed of the robot.
     */
    public static double getRotationSpeed() {
        return rotationSpeed;
    }

    /**
     * Sets the curve function for directional inputs (translations).
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void setTranslationCurve(BiFunction<Double, Double, Double> curve) {
        translationCurve = curve;
    }

    /**
     * Gets the current curve used for directional inputs.
     */
    public static BiFunction<Double, Double, Double> getTranslationCurve() {
        return translationCurve;
    }

    /**
     * Temporarily sets the curve function for directional inputs (translations).
     * This action will atomatically be undone after calling a run method.
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void tempTranslationCurve(BiFunction<Double, Double, Double> curve) {
        if (inactiveTransationCurve != null) {
            translationCurve = curve;
            return;
        }

        inactiveTransationCurve = translationCurve;
        translationCurve = curve;
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
        rotationCurve = curve;
    }

    /**
     * Gets the Function currently used for turning.
     */
    public static Function<Double, Double> getRotationCurve() {
        return rotationCurve;
    }

    /**
     * Temporarily sets the curve function for turn inputs. Undone after running.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void tempRotationCurve(Function<Double, Double> curve) {
        if (inactiveRotationCurve != null) {
            rotationCurve = curve;
            return;
        }

        inactiveRotationCurve = rotationCurve;
        rotationCurve = curve;
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
        var x = translationCurve.apply(translationX, translationY);
        var y = translationCurve.apply(translationY, translationX);
        rotation = rotationCurve.apply(rotation);
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
        translationSpeedX = translationX;
        translationSpeedY = translationY;
        rotationSpeed = rotation;

        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        boolean holdPos = false;

        switch (mode) {
            case HEADLESS: {
                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case RELATIVE: {
                moduleStates = kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed
                    )
                ); 
                break;
            }

            case ROCK: {
                assert moduleStates.length == modules.length;
                holdPos = true;

                for (int i = 0; i < modules.length; i++) {
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
                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        -setAngleController.calculate(
                            Pigeon.getYaw().radians(),
                            setAngle.radians()
                        ), 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case AIMBOT_ROTATION: {
                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        Aimbot.calculateTurn(), 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case AIMBOT: {
                moduleStates = kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                        translationSpeedX,
                        Aimbot.isCentered()
                            ? Aimbot.calculateMovement()
                            : 0.0,
                        Aimbot.calculateTurn()
                    )
                );
                break;
            }

            case TRAJECTORY_FOLLOW: {
                if (autonomousTrajectory == null || autonomousTrajectory.getTotalTimeSeconds() + 0.35 < trajectoryTimer.get()) {
                    translationSpeedX = 0.0;
                    translationSpeedY = 0.0;
                    rotationSpeed = 0.0;

                    moduleStates = kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(0.0, 0.0, 0.0)
                    );

                    break;
                }

                State state = autonomousTrajectory.sample(trajectoryTimer.get());
                Pose2d setPosition = state.poseMeters;
                Pose2d position = odometry.getPoseMeters();

                translationSpeedX
                    = trajectoryStrafeXController.calculate(
                        position.getY(), 
                        setPosition.getY()
                    );
                translationSpeedY
                    = trajectoryStrafeYController.calculate(
                        position.getX(),
                        -setPosition.getX()
                    );
                rotationSpeed = -trajectoryRotateController.calculate(
                    position.getRotation().getRadians(),
                    setPosition.getRotation().getRadians()
                );

                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );

                break;
            }

            case SIDEWALK_WALK: {
                if (setPathPosition == null) {
                    // Break early if we have no path.

                    moduleStates = kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, 
                            new Rotation2d(Pigeon.getYaw().radians())
                        )
                    );

                    break;
                }

                Pose2d setPosition = setPathPosition.pose;
                Pose2d position = odometry.getPoseMeters();

                translationSpeedX
                    = trajectoryStrafeXController.calculate(position.getX(), setPosition.getX());
                translationSpeedY
                    = trajectoryStrafeYController.calculate(position.getY(), setPosition.getY());
                rotationSpeed = -trajectoryRotateController.calculate(
                    position.getRotation().getRadians(),
                    setPosition.getRotation().getRadians()
                );

                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );

                break;
            }

            case AMP_LINE_UP: {
                translationSpeedX = Math.abs(ampChargeSpeed) > 0.1 
                    ? 0.0
                    : Aimbot.ampLineUpX() * -getTrajectoryCoefficiantY();
                translationSpeedY = ampChargeSpeed * getTrajectoryCoefficiantY();
                setAngle = new Angle().setDegrees(-90.0 * getTrajectoryCoefficiantY());

                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        -setAngleController.calculate(
                            Pigeon.getYaw().radians(),
                            setAngle.radians()
                        ), 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
            }

            // This branch should never be reached as the enum used should never
            // have more than the above possible values.
            default: assert false;
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
            modules[i].setRockMode(holdPos);
            modules[i].run();
        }

        // Record module states

        SwerveDrive.moduleStates = moduleStates;
        
        // Reset temporary states
        
        displayMode = mode;
        
        if (inactiveMode != null) {
            mode = inactiveMode;
            inactiveMode = null;
        }
        
        if (inactiveTransationCurve != null) {
            translationCurve = inactiveTransationCurve;
            inactiveTransationCurve = null;
        }

        if (inactiveRotationCurve != null) {
            rotationCurve = inactiveRotationCurve;
            inactiveRotationCurve = null;
        }
    }

    /**
     * Record states for logging and displays like AdvantageKit.
     */
    public static void recordStates() {
        SwerveModuleState[] measuredState = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            measuredState[i] = modules[i].getActualState();
        }

        Logger.recordOutput("Swerve Module States Measured", measuredState);
        Logger.recordOutput("Swerve Module States", moduleStates);

        if (setPathPosition != null) {
            Logger.recordOutput("Swerve Drive Target Pose", setPathPosition.pose);
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

        Logger.recordOutput("Swerve Chassis Speeds Output Vx", chassisSpeedsOutput.vxMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Output Vy", chassisSpeedsOutput.vyMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Output Tau", chassisSpeedsOutput.omegaRadiansPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Calculated Vx", chassisSpeedsCalculated.vxMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Calculated Vy", chassisSpeedsCalculated.vyMetersPerSecond);
        Logger.recordOutput("Swerve Chassis Speeds Calculated Tau", chassisSpeedsCalculated.omegaRadiansPerSecond);

        Logger.recordOutput("Swerve Trajectory Coefficiant X", trajectoryCoefficiantX);
        Logger.recordOutput("Swerve Trajectory Coefficiant Y", trajectoryCoefficiantY);
    }

    /**
     * Updates module positions and drive odometry.
     */
    public static void updateOdometry() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updatePosition();
            positions[i] = modules[i].getPosition();
        }

        chassisSpeedsOutput = kinematics.toChassisSpeeds(
            modules[0].getDesiredState(),
            modules[1].getDesiredState(),
            modules[2].getDesiredState(),
            modules[3].getDesiredState()
        );

        chassisSpeedsCalculated = kinematics.toChassisSpeeds(
            modules[0].getActualState(),
            modules[1].getActualState(),
            modules[2].getActualState(),
            modules[3].getActualState()
        );

        odometry.update(new Rotation2d(Pigeon.getYaw().radians()), positions);
    }
    
    /**
     * Gets current odometry position.
     */
    public static Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Zeros position entirely, assuming the robot is facing forward, then set
     * the odometry position to the given X and Y components.
     */
    public static void setOdometry(Pose2d pose) {
        //zeroPositions();

        odometry.resetPosition(
            new Rotation2d(Pigeon.getYaw().radians()),
            positions,
            pose
        );
    }
    
    /**
     * Zeros all movement encoder positions.
     */
    public static void zeroPositions() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].zeroPositions();
            positions[i] = modules[i].getPosition();
        }

        odometry.resetPosition(
            new Rotation2d(Pigeon.getYaw().radians()), 
            positions, 
            new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        );
    }

    public static void setAmpChargeSpeed(double speed) {
        ampChargeSpeed = speed;
    }

    /**
     * Gets the average tempurature of all motors on the drive in celsius.
     */
    public static double getAverageMotorTemp() {
        double tempSum = 0.0;

        for (SwerveModule module : modules) {
            tempSum += module.getRotationMotorTemp();
            tempSum += module.getMovementMotorTemp();
        }

        return tempSum / (modules.length * 2.0);
    }

    /**
     * Gets the sum of all applied currents in amps of all motors on the drive.
     */
    public static double getAppliedCurrent() {
        double current = 0.0;

        for (SwerveModule module : modules) {
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

        for (SwerveModule module : modules) {
            percentSum += module.getPercentRatedCurrent();
        }

        return percentSum / (double)modules.length;
    }

    /**
     * Loads a PathWeaver JSON file to be used in the TRAJECTORY_FOLLOW drive
     * mode. This function will set the odometry position to the beginning of
     * trajectory.
     */
    public static void grabPathweaverFile(String filePath) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + filePath);
            autonomousTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            
            Pose2d initialPosition = autonomousTrajectory.getInitialPose();
            initialPosition = new Pose2d(
                -initialPosition.getX(),
                initialPosition.getY(),
                initialPosition.getRotation()
            );

            odometry.resetPosition(
                new Rotation2d(Pigeon.getYaw().radians()),
                positions,
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
        trajectoryTimer.reset();
        trajectoryTimer.start();
    }

    /**
     * Sets the set angle for `SET_ANGLE` drive mode.
     */
    public static void setTargetAngle(Angle angle) {
        setAngle = angle;
    }

    /**
     * Sets the path position for the `SIDEWALK_WALK` mode.
     */
    public static void setTargetPathPosition(PathPosition setPathPosition) {
        PathPosition pose = new PathPosition(
            new Pose2d(
                setPathPosition.pose.getX() * trajectoryCoefficiantX, 
                setPathPosition.pose.getY() * trajectoryCoefficiantY, 
                trajectoryCoefficiantY < 0.0
                    ? new Rotation2d(Angle.TAU - setPathPosition.pose.getRotation().getRadians())
                    : setPathPosition.pose.getRotation()
            ),
            setPathPosition.timeSeconds
        );

        setPathPosition = pose;
    }

    public static boolean withinPositionTolerance() {
        final double TOLERANCE_X = 0.3;
        final double TOLERANCE_Y = 0.3;
        final double TOLERANCE_THETA = 0.1;

        // makes sure angle are within [0, tau)
        double measuredAngle = (getOdometryPose().getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;
        double setAngle = (setPathPosition.pose.getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;

        return Math.abs(getOdometryPose().getX() - setPathPosition.pose.getX()) < TOLERANCE_X
            && Math.abs(getOdometryPose().getY() - setPathPosition.pose.getY()) < TOLERANCE_Y
            && Math.abs(measuredAngle - setAngle) < TOLERANCE_THETA;
    }

    public static void setTrajectoryCoefficiants(double x, double y) {
        trajectoryCoefficiantX = x;
        trajectoryCoefficiantY = y;
    }

    public static double getTrajectoryCoefficiantX() {
        return trajectoryCoefficiantX;
    }

    public static double getTrajectoryCoefficiantY() {
        return trajectoryCoefficiantY;
    }
}
