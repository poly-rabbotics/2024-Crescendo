package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class PathPosition {
    public Pose2d pose;
    public double timeSeconds;

    /**
     * Creates a new `PathPosition`, wrapping the given `Pose2d` and
     * associating it with the given time in seconds.
     */
    public PathPosition(Pose2d pose, double timeSeconds) {
        this.pose = pose;
        this.timeSeconds = timeSeconds;
    }
}
