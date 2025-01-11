package frc.robot.maps.subsystems.patterns;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class AlgaeDestageMap implements LoggableMap<AlgaeDestageMap.Data> {

    public SmartMotorController motor;
    public double spinSpeed;

    public AlgaeDestageMap() {
        this(new SmartMotorController(), 0);
    }

    public AlgaeDestageMap(SmartMotorController motor, double spinSpeed) {
        this.motor = motor;
        this.spinSpeed = spinSpeed;
    }

    // Will add data/logging stuff when we need it

    @Override
    public void updateData(Data data) {
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
    }
}
