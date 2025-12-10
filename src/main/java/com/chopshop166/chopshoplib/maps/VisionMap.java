package com.chopshop166.chopshoplib.maps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionMap {

    /** Camera-based position estimators. */
    public final List<CameraSource> visionSources;

    /** Default constructor. */
    public VisionMap() {
        this(new ArrayList<>());
    }

    /**
     * Constructor.
     * 
     * @param visionSources Any number of vision sources.
     */
    public VisionMap(final CameraSource... visionSources) {
        this(Arrays.asList(visionSources));
    }

    /**
     * Constructor.
     * 
     * @param visionSources A list of vision sources.
     */
    public VisionMap(final List<CameraSource> visionSources) {
        this.visionSources = visionSources;
    }

    /**
     * Update the data in a pose estimator with the poses from all cameras.
     * 
     * @param <T>       Estimator wheel type.
     * @param estimator The WPIlib estimator object.
     */
    public <T> void updateData(Data data) {
        data.targets.clear();
        for (var source : this.visionSources) {
            var results = source.camera.getAllUnreadResults();
            Logger.recordOutput("Sees Tags", !results.isEmpty());
            if (!results.isEmpty()) {
                for (PhotonPipelineResult result : results) {
                    if ((result.multitagResult.isPresent()
                            && result.multitagResult.get().estimatedPose.ambiguity > 0.50)) {
                        continue;
                    }
                    // Put the results measurement into the pose estimator
                    var estimate = source.estimator.update(result);
                    estimate.ifPresent(est -> {
                        Logger.recordOutput("Camera Pose Estimate/" + source.camera.getName(),
                                est.estimatedPose.toPose2d());
                        data.estimator.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                est.timestampSeconds);
                    });
                }
                // Now copy the targets that are found
                PhotonPipelineResult latestResult = results.get(results.size() - 1);
                for (var target : latestResult.targets) {
                    int targetID = target.getFiducialId();

                    target.bestCameraToTarget = target.bestCameraToTarget.plus(source.robotToCam);
                    Transform3d reefToRobot = new Transform3d(target.bestCameraToTarget.getTranslation(),
                            target.bestCameraToTarget.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI)));
                    target.bestCameraToTarget = reefToRobot;

                    if (data.targets.containsKey(target.getFiducialId())) {
                        data.targets.get(targetID).add(target);
                    } else {
                        ArrayList<PhotonTrackedTarget> targetList = new ArrayList<>();
                        targetList.add(target);
                        data.targets.put(targetID, targetList);
                    }

                }
            }
        }
    }

    public static class Data {
        public SwerveDrivePoseEstimator estimator;
        public Map<Integer, List<PhotonTrackedTarget>> targets = new HashMap<>();
    }
}
