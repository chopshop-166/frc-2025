package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class AlgaeDestageMap implements LoggableMap<AlgaeDestageMap.Data> {

    public SmartMotorController motor;

    public AlgaeDestageMap() {
        this(new SmartMotorController());
    }

    public AlgaeDestageMap(SmartMotorController motor) {
        this.motor = motor;
    }

    // Will add data/logging stuff when we need it

    @Override
    public void updateData(Data data) {
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
    }
}
