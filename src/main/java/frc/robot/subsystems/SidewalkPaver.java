package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class SidewalkPaver {
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

    // time at start is assumed to be a zero.
    private Pose2d startPose;
    private PathPosition[] path;
    private int index = 0;

    /**
     * Creates a new `SidewalkPaver` that with assume it starts at the given
     * `startPose`, and may then be used to instruct the drive train to move to
     * the given `path` in the given order.
     */
    public SidewalkPaver(Pose2d startPose, PathPosition... path) {
        this.startPose = startPose;
        this.path = path;
    }

    /**
     * Gets the assumed starting position of this `SidewalkPaver`.
     */
    public Pose2d startPose() {
        return startPose;
    }

    /**
     * Gets the next position along the path of this `SidewalkPaver`, the 
     * current position is tracked internally in an iterator-style fashion.
     */
    public PathPosition nextPathPose() {
        var pathPose = path[index];
        index++;
        return pathPose;
    }

    /**
     * Gives points based on time control. Unlike `nextPathPose` this function
     * does not require that points be given in order, you should do so anyways
     * since its dumb not to, but its not required.
     * 
     * This function will never return `null`.
     */
    public PathPosition pathPoseAtTime(double timeSeconds) {
        PathPosition leastHigherPose = null;

        for (PathPosition p : path) {
            // Consider past points invalid.
            if (p.timeSeconds < timeSeconds) {
                continue;
            }

            // If aprox. equal to the requested time just return it.
            if (Math.abs(p.timeSeconds - timeSeconds) < 1e-8) {
                return p;
            }

            // Take the new pose only if it is closer to the given time and in
            // the future. This comparrison can be made since all points in time
            // before the given time are considered invalid, making a lesser
            // time always closer to te given time.
            if (leastHigherPose.timeSeconds < p.timeSeconds) {
                leastHigherPose = p;
            }
        }

        // If no future point exists give the latest point.
        if (leastHigherPose == null) {
            for (PathPosition p : path) {
                if (p.timeSeconds > leastHigherPose.timeSeconds) {
                    leastHigherPose = p;
                }
            }
        }

        assert leastHigherPose != null;
        return leastHigherPose;
    }
}
