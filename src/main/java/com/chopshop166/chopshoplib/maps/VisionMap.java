package com.chopshop166.chopshoplib.maps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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
            if (!results.isEmpty()) {
                PhotonPipelineResult latestResult = results.get(results.size() - 1);
                // Put the results measurement into the pose estimator
                var estimate = source.estimator.update(latestResult);
                estimate.ifPresent(est -> {
                    data.estimator.addVisionMeasurement(est.estimatedPose.toPose2d(),
                            est.timestampSeconds);
                });
                // Now copy the targets that are found
                for (var target : latestResult.targets) {
                    int targetID = target.getFiducialId();
                    Logger.recordOutput("target" + targetID, target.bestCameraToTarget);
                    Transform3d robotToCam = new Transform3d(-source.robotToCam.getX(), -source.robotToCam.getY(),
                            source.robotToCam.getZ(),
                            new Rotation3d(0,
                                    0, source.robotToCam.getRotation().getZ() + Math.PI));
                    Logger.recordOutput("Fucking Z", -source.robotToCam.getRotation().getZ());
                    target.bestCameraToTarget = target.bestCameraToTarget.plus(robotToCam);
                    Logger.recordOutput("targetOffset" + targetID, target.bestCameraToTarget);
                    Logger.recordOutput("blank transform test", new Transform2d());
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
