package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class SidewalkPaver {
    private static final double POSITION_TOLERANCE = 5.0 / 100.0;

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
        if (index >= path.length) {
            return path[path.length - 1];
        }

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
        PathPosition leastHigherPose = path[0];
        
        // Get the furure most pose.
        for (PathPosition p : path) {
            if (p.timeSeconds > leastHigherPose.timeSeconds) {
                leastHigherPose = p;
            }
        }
        
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
            if (leastHigherPose.timeSeconds > p.timeSeconds) {
                leastHigherPose = p;
            }
        }

        assert leastHigherPose != null;
        return leastHigherPose;
    }

    /**
     * Gets the next position if within tolerance of the current set position,
     * otherwise returns the same set position as before.
     */
    public PathPosition nextPoseIfComplete(Pose2d currentPose) {
        var currentSetPose = path[index];

        var distX = currentPose.getX() - currentSetPose.pose.getX();
        var distY = currentPose.getY() - currentSetPose.pose.getX();
        var distance = Math.sqrt(distX * distX + distY * distY);

        if (distance < POSITION_TOLERANCE) {
            return nextPathPose();
        }

        return currentSetPose;
    }

    public boolean isComplete() {
        return index >= path.length;
    }
}
