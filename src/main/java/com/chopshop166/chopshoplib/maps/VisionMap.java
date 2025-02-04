package com.chopshop166.chopshoplib.maps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
        for (var source : this.visionSources) {
            var results = source.camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                var estimate = source.estimator.update(results.get(results.size() - 1));
                for (var result : results) {
                    data.targets = result.targets;
                }
                estimate.ifPresent(est -> {
                    data.estimator.addVisionMeasurement(est.estimatedPose.toPose2d(),
                            est.timestampSeconds);
                });
            }
        }
    }

    public static class Data<T> {
        PoseEstimator<T> estimator;
        List<PhotonTrackedTarget> targets;
    }
}
