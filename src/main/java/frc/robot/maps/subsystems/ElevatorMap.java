package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

public class ElevatorMap implements LoggableMap<ElevatorMap.Data> {

    public final SmartMotorController leftMotor;
    public final SmartMotorController rightMotor;
    public final IEncoder encoder;
    public final double conversionRate;

    public ElevatorMap() {
        this(new SmartMotorController(), new SmartMotorController(), new MockEncoder(), 1);
    }

    public ElevatorMap(SmartMotorController leftMotor, SmartMotorController rightMotor, IEncoder encoder,
            double conversionRate) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.encoder = encoder;
        this.conversionRate = conversionRate;
    }

    @Override
    public void updateData(Data data) {
        data.leftMotor.updateData(leftMotor);
        data.rightMotor.updateData(rightMotor);
        data.heightAbsInches = encoder.getAbsolutePosition() * conversionRate; // need to do math to figure out the
                                                                               // right value
    }

    public static class Data extends DataWrapper {
        public MotorControllerData leftMotor = new MotorControllerData();
        public MotorControllerData rightMotor = new MotorControllerData();
        public double heightAbsInches;
    }
}
