package frc.robot.systems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PosEstimator {
    private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    private static final Transform3d BOT_TO_CAM_TRANSFORM = new Transform3d(); // TODO: Fill from CAD.

    private static final PhotonPoseEstimator photon = new PhotonPoseEstimator(FIELD_LAYOUT, POSE_STRATEGY, BOT_TO_CAM_TRANSFORM);

    /**
     * The current estimation, initial position, or overwritten reference position.
     *
     * It WILL be assumed that this is the BEST estimate of the current postion, second only to a
     * value given by photon vision. This value will also be the value given until as a position
     * and will only be updated upon a call to `PosEstimator.update()`.
     */
    private static Pose3d referencePosition = new Pose3d();

    /**
     * Returns the last updated position value. If `PosEstimator.update()` has not been called since
     * the last call to `PosEstimator.setReferencePosition()`, or if no targets were available 
     * during the last call to `PosEstimator.update()`, then the value given as a reference position
     * will be returned here.  
     */
    public static Pose3d getPosEstimate() {
        return referencePosition;
    }

    /**
     * Sets the internal reference position. This should be used to set an initial position, or
     * override a reference postion before calling `PosEstimator.update()`.
     *
     * This function will only have an effect once `PosEstimator.update()` is called AFTER this
     * function is called.
     */
    public static void setReferencePosition(Pose3d refPos) {
        referencePosition = refPos;
    }

    /**
     * Updates the current position estimate. This polls a value from photon vision, and if one is
     * returned updates the stored position and odometry position with it. Otherwise odometry is 
     * used to provide an X and Y position, with Z being retained from the last succesful photon
     * vision poll, and rotations are grabbed from the gyro.
     */
    public static void update() {
        Optional<EstimatedRobotPose> robotPosEstimate = estimatePostion(referencePosition);
        Pose2d odometryEstimate = SwerveDrive.getOdometryPose();

        // TODO: Verify that no tranformations need be done on pigeon values
        Rotation3d fallbackRotation = new Rotation3d(
            Pigeon.getRoll().radians(), 
            Pigeon.getPitch().radians(),
            Pigeon.getYaw().radians()
        );

        Pose3d fallbackPos = new Pose3d(
            odometryEstimate.getX(), 
            odometryEstimate.getY(), 
            referencePosition.getZ(), 
            fallbackRotation
        );

        Optional<Pose3d> posEstimate = robotPosEstimate.map((pos) -> pos.estimatedPose);
        posEstimate.ifPresent(PosEstimator::updateOdometry);
        referencePosition = posEstimate.orElse(fallbackPos);
    }

    /**
     * Update the drive's odometry with the given position, this function does not use the class's 
     * state. This discards the Z component of both rotation and displacement as odometry tracks
     * only two dimensions.
     */
    private static void updateOdometry(Pose3d pos) {
        Rotation2d rot = new Rotation2d(pos.getRotation().getX(), pos.getRotation().getY());
        Pose2d pos2d = new Pose2d(pos.getX(), pos.getY(), rot); 
        SwerveDrive.setOdometry(pos2d);
    }

    /**
     * Estimate the current robot position using only photon vision. Should be provided a reference 
     * position which is usually the last value given by this function.
     *
     * This function returns no value when targets or cameras are unavailable.
     */
    private static Optional<EstimatedRobotPose> estimatePostion(Pose3d refPos) {
        photon.setReferencePose(refPos);
        photon.setLastPose(refPos);
        return photon.update();
    }
}
