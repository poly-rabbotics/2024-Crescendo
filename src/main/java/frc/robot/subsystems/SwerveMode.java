// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public enum SwerveMode {
    /**
     * Moves relative to the driver station, pressing the joystick away from
     * you should always move the robot away from you.
     */
    HEADLESS,

    /**
     * Pressing the joystick forward moves the robot froward, regardless of 
     * position relative to driver.
     */
    RELATIVE,

    /**
     * Rock mode, resists movement and holds position.
     */
    ROCK,

    /**
     * Operates as though in `HEADLESS` but runs rotation off of a setpoint 
     * rather than by manual speed input.
     */
    SET_ANGLE,

    /**
     * Uses rotation or the drive, but not translation, to aim for the driver.
     */
    AIMBOT_ROTATION,

    /**
     * Uses functions of the `Aimbot` class to both center the target and then
     * aproach it using swerve, taking over both the y-axis and turning.
     */
    AIMBOT,

    /**
     * Follows a set trajectory, as loaded previously from a JSON file, which
     * should have been exported from PathWeaver.
     */
    TRAJECTORY_FOLLOW,

    /**
     * Like `TRAJECTORY_FOLLOW` but for `SidewalkPaver`.
     */
    SIDEWALK_WALK,

    /**
     * Lines up to amp
     */
    AMP_LINE_UP,
}
