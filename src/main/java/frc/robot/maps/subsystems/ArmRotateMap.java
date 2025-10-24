package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import yams.mechanisms.config.ArmConfig;

public record ArmRotateMap(ArmConfig config, PresetValue armRotatePreset) {

    public enum ArmRotatePresets {

        OFF,

        INTAKE,

        SCOREL1,

        SCOREL1_TAKETWO,

        SCOREL2,

        SCOREL3,

        SCOREL4,

        SCOREL4_AUTO,

        OUT,

        STOW,

        ALGAE,

        HOLD

    }

    @FunctionalInterface
    public interface PresetValue extends ToDoubleFunction<ArmRotatePresets> {
    }

    public ArmRotateMap() {
        this(null, p -> Double.NaN);
    }
}