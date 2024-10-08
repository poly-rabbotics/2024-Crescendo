// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.time.Clock;
import java.time.Instant;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.SmartPrintable;
import frc.robot.subsystems.Angle;

/*
 * Manages the robot's pigeon.
 */
public class Pigeon extends SmartPrintable {
    private static final int PIGEON_CAN_ID = 0;
    private static final Pigeon instance = new Pigeon(PIGEON_CAN_ID);

    private final Pigeon2 pigeon;
    private final OrientationalChangeCalculator angularChangeCalculator;
    
    private OrientationalChange changePerSecond = new OrientationalChange(
        new Angle().setDegrees(0.0),
        new Angle().setDegrees(0.0),
        new Angle().setDegrees(0.0)
    );

    private Pigeon(int canID) {
        super();
        pigeon = new Pigeon2(canID);
        angularChangeCalculator = new OrientationalChangeCalculator(this);
    }

    /**
     * Updates stored values of the Piegon.
     */
    public static void update() {
        instance.angularChangeCalculator.run();
    }

    /**
     * Sets "relative forward" to the current position. This will make getting
     * the relative rotation always return a rotation relative to the rotation
     * the robot is in at the point this method is called.
     */
    public static void setFeildZero() {
        instance.pigeon.setYaw(0.0);
    }

    public static void setFeildOrientation(Angle value) {
        instance.pigeon.setYaw(value.degrees());
    }

    /**
     * Gets the change per second in the orientation of the Pigeon.
     */
    public static OrientationalChange getChangePerSecond() {
        return instance.changePerSecond;
    }

    /**
     * Gets Yaw.
     */
    public static Angle getYaw() {
        return new Angle().setDegrees(instance.pigeon.getYaw().getValue() % 360.0);
    }

    /**
     * Gets pitch.
     */
    public static Angle getPitch() {
        return new Angle().setDegrees(instance.pigeon.getPitch().getValue());
    }

    /**
     * Gets roll.
     */
    public static Angle getRoll() {
        return new Angle().setDegrees(instance.pigeon.getRoll().getValue());
    }

    /**
     * Gets the X-axis acceleration as a coefficiant to the acceleration of 
     * gravity.
     */
    public static double getAccelerationX() {
        return instance.pigeon.getAccelerationX().getValueAsDouble();
    }
    
    /**
     * Gets the Y-axis acceleration as a coefficiant to the acceleration of 
     * gravity.
     */
    public static double getAccelerationY() {
        return instance.pigeon.getAccelerationY().getValueAsDouble();
    }
    
    /**
     * Gets the Z-axis acceleration as a coefficiant to the acceleration of 
     * gravity.
     */
    public static double getAccelerationZ() {
        return instance.pigeon.getAccelerationZ().getValueAsDouble();
    }

    public static void recordState() {
        Logger.recordOutput("Pigeon Yaw (Radians)", getYaw().radians());
        Logger.recordOutput("Pigeon Yaw (Degrees)", getYaw().degrees());
        Logger.recordOutput("Pigeon Roll (Radians)", getRoll().radians());
        Logger.recordOutput("Pigeon Roll (Degrees)", getRoll().degrees());
        Logger.recordOutput("Pigeon Pitch (Radians)", getPitch().radians());
        Logger.recordOutput("Pigeon Pitch (Degrees)", getPitch().degrees());

        Logger.recordOutput("Pigeon Yaw/Sec (Radians)", getChangePerSecond().yawPerSecond.radians());
        Logger.recordOutput("Pigeon Yaw/Sec (Degrees)", getChangePerSecond().yawPerSecond.degrees());
        Logger.recordOutput("Pigeon Roll/Sec (Radians)", getChangePerSecond().rollPerSecond.radians());
        Logger.recordOutput("Pigeon Roll/Sec (Degrees)", getChangePerSecond().rollPerSecond.degrees());
        Logger.recordOutput("Pigeon Pitch/Sec (Radians)", getChangePerSecond().pitchPerSecond.radians());
        Logger.recordOutput("Pigeon Pitch/Sec (Degrees)", getChangePerSecond().pitchPerSecond.degrees());

        Logger.recordOutput("Pigeon Acceleration X (g)", getAccelerationX());
        Logger.recordOutput("Pigeon Acceleration Y (g)", getAccelerationY());
        Logger.recordOutput("Pigeon Acceleration Z (g)", getAccelerationZ());
    }
    
    @Override
    public void print() {
        SmartDashboard.putString("Pigeon Yaw", getYaw().toString());
        SmartDashboard.putString("Pigeon Pitch", getPitch().toString());
        SmartDashboard.putString("Pigeon Roll", getRoll().toString());

        OrientationalChange change = getChangePerSecond();

        SmartDashboard.putString("Pigeon Yaw/Sec", change.yawPerSecond.toString());
        SmartDashboard.putString("Pigeon Pitch/Sec", change.pitchPerSecond.toString());
        SmartDashboard.putString("Pigeon Roll/Sec", change.rollPerSecond.toString());
    }

    /**
     * Represents the change per second in orientation as gathered and 
     * calculated from the Pigeon.
     */
    public static class OrientationalChange implements Cloneable {
        public final Angle yawPerSecond;
        public final Angle rollPerSecond;
        public final Angle pitchPerSecond;

        /**
         * Clones all given angles allowing the caller to mutate the passed in
         * references freely, without modifying this class's state.
         */
        private OrientationalChange(Angle yaw, Angle roll, Angle pitch) {
            yawPerSecond = yaw.clone();
            rollPerSecond = roll.clone();
            pitchPerSecond = pitch.clone();
        }

        @Override
        public OrientationalChange clone() {
            return new OrientationalChange(
                yawPerSecond.clone(),
                rollPerSecond.clone(),
                pitchPerSecond.clone()
            );
        }
    }

    /*
     * This is made a subclass to make sure no one gets confused and tries to
     * run the whole Pigeon on a thread in Robot.java
     */
    private class OrientationalChangeCalculator implements Runnable {
        private final Pigeon pigeon;  // Reference to containing pigeon.
        private final Clock clock;    // Clock used to get current instant.

        private Instant recordedInstant;
        
        private Angle previousYaw;
        private Angle previousRoll;
        private Angle previousPitch;

        private OrientationalChangeCalculator(Pigeon pigeon) {
            this.pigeon = pigeon;
            clock = Clock.systemDefaultZone();
            recordedInstant = clock.instant();
            
            previousYaw = new Angle().setDegrees(pigeon.pigeon.getYaw().getValue());
            previousRoll = new Angle().setDegrees(pigeon.pigeon.getRoll().getValue());
            previousPitch = new Angle().setDegrees(pigeon.pigeon.getPitch().getValue());
        }

        @Override
        public void run() {
            Instant previousInstant = recordedInstant;

            Angle yaw = new Angle().setDegrees(pigeon.pigeon.getYaw().getValue());
            Angle roll = new Angle().setDegrees(pigeon.pigeon.getRoll().getValue());
            Angle pitch = new Angle().setDegrees(pigeon.pigeon.getPitch().getValue());             

            recordedInstant = clock.instant();
            
            double differenceSeconds = (double)(recordedInstant.toEpochMilli() - previousInstant.toEpochMilli()) / 1000.0;

            Angle changeYaw = yaw
                .sub(previousYaw)
                .div(differenceSeconds);
            Angle changeRoll = roll
                .sub(previousRoll)
                .div(differenceSeconds);
            Angle changePitch = pitch
                .sub(previousPitch)
                .div(differenceSeconds);

            instance.changePerSecond = new OrientationalChange(changeYaw, changeRoll, changePitch);

            previousYaw = yaw;
            previousRoll = roll;
            previousPitch = pitch;
        }
    }
}
