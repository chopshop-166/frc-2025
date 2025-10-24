package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import yams.mechanisms.config.ElevatorConfig;

public record ElevatorMap(ElevatorConfig config, PresetValues presetValues) {

    public enum ElevatorPresets {
        OFF,

        ZEROING,

        INTAKE,

        SCOREL1,

        SCOREL1_TAKETWO,

        SCOREL2,

        ALGAEL2,

        SCOREL3,

        ALGAEL3,

        SCOREL4,

        HIGHESTPOINT,

        STOW,

        HOLD
    }

    public interface PresetValues extends ToDoubleFunction<ElevatorPresets> {
    }

    public ElevatorMap() {
        this(null, p -> Double.NaN);
    }
}
