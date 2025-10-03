package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;

import yams.mechanisms.positional.Elevator;

public class ElevatorMap implements LoggableMap<ElevatorMap.Data> {

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

    public final Elevator elevator;
    public final PresetValues presetValues;

    public ElevatorMap() {
        this(null, p -> Double.NaN);
    }

    public ElevatorMap(Elevator elevator, PresetValues presetValues) {
        this.elevator = elevator;
        this.presetValues = presetValues;
    }

    @Override
    public void updateData(Data data) {
    }

    public static class Data extends DataWrapper {
    }
}
