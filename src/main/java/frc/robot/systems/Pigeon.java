// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.time.Clock;
import java.time.Instant;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

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
    private final ScheduledExecutorService changeRateThread;
    
    private OrientationalChange changePerSecond;

    private Pigeon(int canID) {
        super();
        pigeon = new Pigeon2(canID);

        changePerSecond = new OrientationalChange(
            new Angle().setDegrees(0.0),
            new Angle().setDegrees(0.0),
            new Angle().setDegrees(0.0)
        );

        /* 
         * Starts the thread so that it calls 'run()' every 40 ms (25hz). This
         * automatically updates the changePerSecond feilds at that rate.
         */

        OrientationalChangeCalculator angularChangeCalculator = new OrientationalChangeCalculator(this);
        changeRateThread = Executors.newSingleThreadScheduledExecutor();
        changeRateThread.scheduleAtFixedRate(angularChangeCalculator, 0, 40, TimeUnit.MILLISECONDS);
    }

    /**
     * Sets "relative forward" to the current position. This will make getting
     * the relative rotation always return a rotation relative to the rotation
     * the robot is in at the point this method is called.
     */
    public static void setFeildZero() {
        instance.pigeon.setYaw(0.0);
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
         * references freely, withoutr modifying this class' state.
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
