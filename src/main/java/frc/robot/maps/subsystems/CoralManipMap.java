package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class CoralManipMap implements LoggableMap<CoralManipMap.Data> {
    public final SmartMotorController motor;
    public final BooleanSupplier sensor;

    public CoralManipMap() {
        this(new SmartMotorController(), () -> false);
    }

    public CoralManipMap(SmartMotorController motor, BooleanSupplier sensor) {
        this.motor = motor;
        this.sensor = sensor;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.gamePieceDetected = sensor.getAsBoolean();

    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public boolean gamePieceDetected;
    }

}
