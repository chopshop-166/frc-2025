package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.sensors.IEncoder;

public class FunnelMap implements LoggableMap<FunnelMap.Data> {

    public SmartMotorController motor;
    public final IEncoder encoder;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;

    public FunnelMap(SmartMotorController motor, IEncoder encoder, ValueRange hardLimits, ValueRange softLimits) {
        this.motor = motor;
        this.encoder = encoder;
        this.softLimits = softLimits;
        this.hardLimits = hardLimits;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.rotationAbsAngleDegrees = encoder.getAbsolutePosition();
        data.rotatingAngleVelocity = encoder.getRate();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
        public double rotatingAngleVelocity;
    }

}
