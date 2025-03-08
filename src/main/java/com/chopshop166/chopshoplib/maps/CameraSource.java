package com.chopshop166.chopshoplib.maps;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** Simplified wrapper around a camera with a pose estimator. */
public class CameraSource {
    /** The default field, loaded for convenience. */
    public static final AprilTagFieldLayout DEFAULT_FIELD = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);
    /** The camera object. */
    public final PhotonCamera camera;
    /** The pose estimator. */
    public final PhotonPoseEstimator estimator;
    /** The robotToCam object. */
    public final Transform3d robotToCam;

    /**
     * Constructor.
     * 
     * @param cameraName The camera's name in Photon Vision.
     * @param robotToCam The offset of the camera from the center of the robot.
     */
    public CameraSource(final String cameraName, final Transform3d robotToCam) {
        this(new PhotonCamera(cameraName), robotToCam);
    }

    /**
     * Constructor.
     * 
     * @param cameraName The camera object.
     * @param robotToCam The offset of the camera from the center of the robot.
     */
    public CameraSource(final PhotonCamera camera, final Transform3d robotToCam) {
        this.camera = camera;
        this.robotToCam = robotToCam;
        this.estimator = new PhotonPoseEstimator(DEFAULT_FIELD,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        this.estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
}
