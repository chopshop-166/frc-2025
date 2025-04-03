package frc.robot;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.chopshop166.chopshoplib.maps.CameraSource;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Vision {

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = CameraSource.DEFAULT_FIELD;

    Map<Integer, Pose2d> BLUE_APRIL_TAGS_REEF_POSITIONS = new HashMap<>();
    Map<Integer, Pose2d> RED_APRIL_TAGS_REEF_POSITIONS = new HashMap<>();

    public enum Branch {
        LEFT_BRANCH(new Transform2d(
                Units.inchesToMeters(18.9), // bot center to reef (w/ bumpers)
                Units.inchesToMeters(-12.94 / 2), // distance between branches div by 2
                Rotation2d.fromDegrees(180))),
        RIGHT_BRANCH(new Transform2d(
                Units.inchesToMeters(18.9),
                Units.inchesToMeters(12.94 / 2),
                Rotation2d.fromDegrees(180))),
        NONE(null);

        private Transform2d offset;

        private Branch(Transform2d offset) {
            this.offset = offset;
        }

        public Transform2d getOffset() {
            return offset;
        }
    };

    public Vision() {
        for (int i = 6; i <= 11; i++) {
            final int tmpI = i;
            kTagLayout.getTagPose(i).ifPresent(pose -> {
                RED_APRIL_TAGS_REEF_POSITIONS.put(tmpI, pose.toPose2d());
            });
        }

        for (int i = 17; i <= 22; i++) {
            final int tmpI = i;
            kTagLayout.getTagPose(i).ifPresent(pose -> {
                BLUE_APRIL_TAGS_REEF_POSITIONS.put(tmpI, pose.toPose2d());
            });
        }
    }

    private Map<Integer, Pose2d> getOurReef(boolean isBlueAlliance) {
        if (isBlueAlliance) {
            return BLUE_APRIL_TAGS_REEF_POSITIONS;
        }
        return RED_APRIL_TAGS_REEF_POSITIONS;
    }

    public static int getNearestTagIdImpl(Map<Integer, Pose2d> poses, SwerveDrivePoseEstimator estimator) {
        Pose2d robotPose = estimator.getEstimatedPosition();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();
        return Collections.min(
                poses.entrySet(),
                Comparator.comparing((Map.Entry<Integer, Pose2d> entry) -> {
                    return robotTranslation.getDistance(entry.getValue().getTranslation());
                }).thenComparing((Map.Entry<Integer, Pose2d> entry) -> {
                    return Math.abs(robotRotation.minus(entry.getValue().getRotation()).getRadians());
                })).getKey();
    }

    // public Command alignToReefBranch
    // Probably want arguments of leftBranch or rightBranch?
    // Pull reef coords from FieldConstants somehow
    // Find nearest reef apriltag (do we need a different method of estimation
    // rather than global? Might want to just focus on one tag rather than multiple
    // behind the robot. Reef tags are blue 17-22, red 6-11)
    // Use coords and move command to move to either left or right branch
    // (translation to get robotToReef and then move)
    // Need rotation in here somewhere. Logic from rotateTo command, since ideally
    // we move and rotate at the same time

    // Function that takes current robot pose and finds the nearest reef apriltag to
    // it, returning the id
    /////////////////////// After --> and taking the average pose from both cameras?
    ///
    // align to reef branch command part two:
    // 1. Filter list of apriltags and figure out which ones we see
    // 2. filter down to reef apriltags and get poses
    // 3. find best target (nearest tag or tag closest to center of sight? Another
    // way?)
    // 4. now we know that best target plane is a reef side. Apriltags are always in
    // the middle at the same point in relation to the reef side, so then
    // apply translation offset for left or right branch
    // 5. With branch pose, use autoBuilder to maneuver to the branch (angle
    // funky??)

    //

    public void filterReefTags(boolean isBlueAlliance, Map<Integer, List<PhotonTrackedTarget>> targets) {
        var reef = getOurReef(isBlueAlliance);
        for (var key : targets.keySet()) {
            // Pull in keySet from targets HashMap
            // Remove all IDs / keys which aren't our reef
            if (!reef.keySet().contains(key)) {
                targets.remove(key);
            }
        }
    }

    public PhotonTrackedTarget pickBestReefLocation(Map<Integer, List<PhotonTrackedTarget>> targets) {
        // Find the closest april tag target
        // For now we only look at a single camera's data
        // In the future we may want to combine the pose data for the april tags?
        if (targets.size() == 0) {
            return new PhotonTrackedTarget();
        }
        var closestTarget = Collections.min(
                targets.entrySet(),
                Comparator.comparing((Map.Entry<Integer, List<PhotonTrackedTarget>> entry) -> {
                    return entry.getValue().get(0).getBestCameraToTarget().getTranslation().getNorm();
                }));

        return closestTarget.getValue().get(0);
    }

    public Transform2d adjustTranslationForBranch(Transform2d tagToCamera, Branch branch) {
        // Maybe just works?? This should take into account the rotation of the tag I
        // believe
        return tagToCamera.plus(branch.getOffset());
    }

    public int findNearestTagId(boolean isBlueAlliance, SwerveDrivePoseEstimator estimator) {
        return getNearestTagIdImpl(getOurReef(isBlueAlliance), estimator);
    }

    public static Translation2d getReefCenter(boolean isBlueAlliance) {
        Translation3d translation;
        if (isBlueAlliance) {
            Translation3d poseLeft = kTagLayout.getTagPose(18).get().getTranslation();
            Translation3d poseRight = kTagLayout.getTagPose(21).get().getTranslation();
            Logger.recordOutput("Reef Center", "Blue");
            translation = poseLeft.plus(poseRight).div(2);
        } else {
            Translation3d poseLeft = kTagLayout.getTagPose(10).get().getTranslation();
            Translation3d poseRight = kTagLayout.getTagPose(7).get().getTranslation();
            Logger.recordOutput("Reef Center", "Red");
            translation = poseLeft.plus(poseRight).div(2);
        }
        return translation.toTranslation2d();
    }
}
