package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

public class DeepClimbMap implements LoggableMap<DeepClimbMap.Data> {

    public final SmartMotorController motor;
    public final IEncoder encoder;

    public DeepClimbMap() {
        this(new SmartMotorController(), new MockEncoder());
    }

    public DeepClimbMap(SmartMotorController motor, IEncoder encoder) {
        this.motor = motor;
        this.encoder = encoder;
    }

    // Will add data/logging stuff when we need it

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.rotationAbsAngleDegrees = encoder.getAbsolutePosition();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
    }
}