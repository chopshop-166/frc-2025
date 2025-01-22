package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class CoralManipMap implements LoggableMap<CoralManipMap.Data> {
    public SmartMotorController motor;

    public CoralManipMap() {
        this(new SmartMotorController());
    }

    public CoralManipMap(SmartMotorController motor) {
        this.motor = motor;
    }

    // Will add data/logging stuff when we need it

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
    }

}
