package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

public class FunnelMap implements LoggableMap<FunnelMap.Data> {

    public SmartMotorController motor;
    public final IEncoder encoder;
    public double driveSpeed;

    public FunnelMap() {
        this.motor = new SmartMotorController();
        this.encoder = new MockEncoder();
        this.driveSpeed = 0;
    }

    public FunnelMap(SmartMotorController motor, IEncoder encoder, double driveSpeed) {
        this.motor = motor;
        this.encoder = encoder;
        this.driveSpeed = driveSpeed;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.rotationDistance = encoder.getDistance();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationDistance;
    }

}
