package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import yams.mechanisms.config.ArmConfig;

public class ArmRotateMap {

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

    public final ArmConfig config;
    public final PresetValue armRotatePreset;

    public ArmRotateMap() {
        this(null, p -> Double.NaN);

    }

    public ArmRotateMap(ArmConfig config, PresetValue armRotatePreset) {
        this.config = config;
        this.armRotatePreset = armRotatePreset;
    }
}