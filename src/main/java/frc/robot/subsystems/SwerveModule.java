// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.SmartPrintable;

/** 
 * Class for managing and manipulating a swerve module. 
 */
public class SwerveModule extends SmartPrintable {
    private static final double CONVERSION_FACTOR_ROTATION = Math.toRadians(150 / 7);                         // Rotations to radians.
    private static final double CONVERSION_FACTOR_MOVEMENT = 6.75;                                            // Rotations to meters.
    private static final double CONVERSION_FACTOR_ROTATION_VELOCITY = CONVERSION_FACTOR_ROTATION * (1 / 60);  // RPM to radians per second.
    private static final double CONVERSION_FACTOR_MOVEMENT_VELOCITY = -CONVERSION_FACTOR_MOVEMENT * (1 / 60);  // RPM to meters per second.
    private static final double CAN_SPARK_MAX_RATED_AMPS = 60.0;

    private static final double PID_P = 0.5;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double ROCK_PID_P = 0.05;
    private static final double ROCK_PID_I = 0.0;
    private static final double ROCK_PID_D = 0.0;

    private static final double VELOCITY_CONTROL_PID_P = 0.05;
    private static final double VELOCITY_CONTROL_PID_I = 0.00;
    private static final double VELOCITY_CONTROL_PID_D = 0.00;

    private final CANSparkMax rotationMotor;  // The motor responsible for rotating the module.
    private final CANSparkMax movementMotor;  // The motor responsible for creating movement in the module.
    private final CANcoder angularEncoder;    // Cancoder responsible for tracking the angle of the module.

    private final RelativeEncoder rotationEncoder; // Relative encoder for tracking rotational movement.
    private final RelativeEncoder movementEncoder; // Relative encoder for tracking translational movement.

    private final PIDController rotationController;
    private final PIDController rockController;

    private final RelativePosition physicalPosition;
    private final Angle canCoderOffset;

    private SwerveModuleState desiredState;
    private SwerveModulePosition position;

    // Set to NaN if not in rock mode, NaN does not equal itself by definition
    // (see some IEEE standard or something) and so this is how rock mode is 
    // checked.
    private double rockPos = Double.NaN;

    private enum RelativePosition {
        FRONT_RIGHT (  1.0,  1.0 ),
        FRONT_LEFT  ( -1.0,  1.0 ),
        BACK_LEFT   ( -1.0, -1.0 ),
        BACK_RIGHT  (  1.0, -1.0 );

        private boolean front = true;
        private boolean right = true;

        RelativePosition(double x, double y) {
            right = Math.signum(x) > 0.0;
            front = Math.signum(y) > 0.0;
        }

        public String asString() {
            String str = "";
            
            if (front) {
                str += "Front ";
            } else {
                str += "Back ";
            }

            if (right) {
                str += "Right";
            } else {
                str += "Left";
            }

            return str;
        }

        public static RelativePosition fromTranslation(Translation2d translation) {
            var x_sign = Math.signum(translation.getX()) > 0.0;
            var y_sign = Math.signum(translation.getY()) > 0.0;

            if (x_sign && y_sign) {
                return FRONT_RIGHT;
            } else if (x_sign) {
                return BACK_RIGHT;
            } else if (y_sign) {
                return FRONT_LEFT;
            } 
                
            return BACK_LEFT;
        }
    }

    public void run() {
        double angularPosition = angularEncoder.getPosition().getValue() * Angle.TAU;
        double currentPosition = (angularPosition + canCoderOffset.radians()) % Angle.TAU;

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(currentPosition));
    
        if (rockPos != rockPos) {
            //movementMotor
            //    .getPIDController()
            //    .setReference(
            //        state.speedMetersPerSecond / CONVERSION_FACTOR_MOVEMENT_VELOCITY, 
            //        CANSparkBase.ControlType.kVelocity
            //    );
            movementMotor.set(state.speedMetersPerSecond);
        } else {
            movementMotor.set(rockController.calculate(getDistanceTraveled(), rockPos));
        }

        double calculation = rotationController.calculate(currentPosition, (state.angle.getRadians() + Angle.TAU) % Angle.TAU);
        rotationMotor.set(calculation);

        position.angle = new Rotation2d(angularPosition);
        position.distanceMeters = movementEncoder.getPosition();
    }
    
    public void updatePosition() {
        position = new SwerveModulePosition(
            movementEncoder.getPosition(), 
            new Rotation2d(angularEncoder.getPosition().getValue() * Angle.TAU)
        );
    }

    public SwerveModule(
        int movementMotorID, 
        int rotationalMotorID, 
        int canCoderID, 
        Angle canCoderOffset, 
        Translation2d physicalPosition
    ) {
        super();
        
        this.physicalPosition = RelativePosition.fromTranslation(physicalPosition);
        this.canCoderOffset = canCoderOffset.clone();

        rotationMotor = new CANSparkMax(rotationalMotorID, CANSparkMax.MotorType.kBrushless);
        rotationMotor.setInverted(true);
        rotationMotor.setSmartCurrentLimit(30);
        
        movementMotor = new CANSparkMax(movementMotorID, CANSparkMax.MotorType.kBrushless);
        movementMotor.setInverted(false);
        movementMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        movementMotor.setSmartCurrentLimit(40);

        angularEncoder = new CANcoder(canCoderID);
        angularEncoder.getConfigurator().apply(new CANcoderConfiguration());

        rotationEncoder = rotationMotor.getEncoder();
        rotationEncoder.setPosition(angularEncoder.getPosition().getValue() * Angle.TAU);
        rotationEncoder.setPositionConversionFactor(CONVERSION_FACTOR_ROTATION);
        rotationEncoder.setVelocityConversionFactor(CONVERSION_FACTOR_ROTATION_VELOCITY);

        movementEncoder = movementMotor.getEncoder();
        movementEncoder.setPosition(0.0);
        movementEncoder.setPositionConversionFactor(CONVERSION_FACTOR_MOVEMENT);
        //movementEncoder.setVelocityConversionFactor(CONVERSION_FACTOR_MOVEMENT_VELOCITY);

        movementMotor.getPIDController().setP(VELOCITY_CONTROL_PID_P);
        movementMotor.getPIDController().setI(VELOCITY_CONTROL_PID_I);
        movementMotor.getPIDController().setD(VELOCITY_CONTROL_PID_D);
        
        rotationController = new PIDController(PID_P, PID_I, PID_D);
        rotationController.enableContinuousInput(0.0, Angle.TAU);
        rotationController.setTolerance(0.01);

        rockController = new PIDController(ROCK_PID_P, ROCK_PID_I, ROCK_PID_D);
        rockController.setTolerance(1);

        position = new SwerveModulePosition(
            movementEncoder.getPosition(), 
            new Rotation2d(angularEncoder.getPosition().getValue() * Angle.TAU)
        );

        desiredState = new SwerveModuleState();
    }

    /**
     * Sets the desired module state for this module. This must be run 
     * repeatedly to continue PID calculations.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    /**
     * True if the module should be in rock mode.
     */
    public void setRockMode(boolean shouldHold) {
        if (!shouldHold) {
            rockPos = Double.NaN;
        } else if (rockPos != rockPos) {
            rockPos = getDistanceTraveled();
        }
    }

    /**
     * True if in rock mode.
     */
    public boolean inRockMode() {
        return rockPos == rockPos;
    }

    /**
     * Gets distance traveled, should not be used for ablsolute distances as 
     * this function currently makes no guarantee as to the starting position
     * of the module. (This can be mitigated if you zero positions, but it will
     * interupt odometry).
     */
    public double getDistanceTraveled() {
        return movementEncoder.getPosition();
    }

    /**
     * Gets the reported tempurature of the rotation motor in celsius.
     */
    public double getRotationMotorTemp() {
        return rotationMotor.getMotorTemperature();
    }

    /**
     * Gets the reported tempurature of the movement motor in celsius.
     */
    public double getMovementMotorTemp() {
        return movementMotor.getMotorTemperature();
    }

    /**
     * Gets the power being outputted by the rotation motor's controller in amps.
     */
    public double getRotationMotorCurrent() {
        return rotationMotor.getOutputCurrent();
    }

    /**
     * Gets the power being outputted by the movement motor's controller in amps.
     */
    public double getMovementMotorCurrent() {
        return movementMotor.getOutputCurrent();
    }

    /**
     * Gets the sum of all motor's current in amps.
     */
    public double getAppliedCurrent() {
        return getRotationMotorCurrent() + getMovementMotorCurrent();
    }

    /**
     * Gets the percentage of the maximum rated amperage of the motor 
     * controllers currently being hit by the module.
     */
    public double getPercentRatedCurrent() {
        return getAppliedCurrent() / (2.0 * CAN_SPARK_MAX_RATED_AMPS);
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position", Math.toDegrees(angularEncoder.getPosition().getValue() * Angle.TAU));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position mod 360", Math.toDegrees(angularEncoder.getPosition().getValue() * Angle.TAU % Angle.TAU));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position + off", Math.toDegrees(angularEncoder.getPosition().getValue() * Angle.TAU + canCoderOffset.radians()));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position + off mod 360", Math.toDegrees((angularEncoder.getPosition().getValue() * Angle.TAU + canCoderOffset.radians()) % Angle.TAU));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position (Distance) ", movementEncoder.getPosition());
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Movement Speed", movementMotor.get());
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Movement Velocity", movementEncoder.getVelocity());
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Desired Velocity", desiredState.speedMetersPerSecond);
    }

    /**
     * Gets the position of the module. 
     */
    public SwerveModulePosition getPosition() {
        return position;
    }

    /**
     * Sets movement position to zero, will mess up odometry.
     */
    public void zeroPositions() {
        movementEncoder.setPosition(0.0);
    }
}
