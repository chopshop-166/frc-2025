package com.chopshop166.chopshoplib.maps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.PoseEstimator;

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
    public <T> void updateData(Data<T> data) {
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
                // data.targets.add(List.copyOf(latestResult.targets));
                for (var target : latestResult.targets) {
                    int targetID = target.getFiducialId();
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

    public static class Data<T> {
        public PoseEstimator<T> estimator;
        public Map<Integer, List<PhotonTrackedTarget>> targets = new HashMap<>();
    }
}
