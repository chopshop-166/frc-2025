package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class DeepClimbMap implements LoggableMap<DeepClimbMap.Data> {

    public final SmartMotorController motor;
    public final BooleanSupplier sensor;

    public DeepClimbMap() {
        this(new SmartMotorController(), () -> true);
    }

    public DeepClimbMap(SmartMotorController motor, BooleanSupplier sensor) {
        this.motor = motor;
        this.sensor = sensor;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.atBottomLimit = sensor.getAsBoolean();

    }
    
    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public boolean atBottomLimit;
    }
}