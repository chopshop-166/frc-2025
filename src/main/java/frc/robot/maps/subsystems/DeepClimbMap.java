package frc.robot.maps.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;

import yams.motorcontrollers.SmartMotorController;

public class DeepClimbMap implements LoggableMap<DeepClimbMap.Data> {

    public final SmartMotorController motor;
    public final BooleanSupplier sensor;

    public DeepClimbMap() {
        this(null, () -> true);
    }

    public DeepClimbMap(SmartMotorController motor, BooleanSupplier sensor) {
        this.motor = motor;
        this.sensor = sensor;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.encoderReading = motor.getMeasurementPosition().in(Inches);
        data.atBottomLimit = sensor.getAsBoolean();

    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public boolean atBottomLimit;
        public double encoderReading;
    }
}