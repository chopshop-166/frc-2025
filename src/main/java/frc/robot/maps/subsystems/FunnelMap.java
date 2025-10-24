package frc.robot.maps.subsystems;

import yams.mechanisms.config.ArmConfig;

public record FunnelMap(ArmConfig armConfig, double driveSpeed) {
    public FunnelMap() {
        this(null, Double.NaN);
    }
}
