package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;

import yams.mechanisms.config.ArmConfig;
import yams.motorcontrollers.SmartMotorController;

public class DeepClimbMap implements LoggableMap<DeepClimbMap.Data> {

    public final SmartMotorController motor;
    public final BooleanSupplier sensor;
    public final ArmConfig config;

    public DeepClimbMap() {
        this(null, () -> true, null);
    }

    public DeepClimbMap(SmartMotorController motor, BooleanSupplier sensor, ArmConfig config) {
        this.motor = motor;
        this.sensor = sensor;
        this.config = config;
    }

    @Override
    public void updateData(Data data) {
        data.atBottomLimit = sensor.getAsBoolean();
    }

    public static class Data extends DataWrapper {
        public boolean atBottomLimit;
    }
}