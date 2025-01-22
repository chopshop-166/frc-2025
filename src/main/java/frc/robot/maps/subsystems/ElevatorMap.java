package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

public class ElevatorMap implements LoggableMap<ElevatorMap.Data> {

    public SmartMotorController leftMotor;
    public SmartMotorController rightMotor;
    public final IEncoder encoder;

    public ElevatorMap() {
        this(new SmartMotorController(), new SmartMotorController(), new MockEncoder());
    }

    public ElevatorMap(SmartMotorController leftMotor, SmartMotorController rightMotor, IEncoder encoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.encoder = encoder;
    }

    // Will add data/logging stuff when we need it

    @Override
    public void updateData(Data data) {
        data.leftMotor.updateData(leftMotor);
        data.rightMotor.updateData(rightMotor);
        data.heightAbsInches = encoder.getAbsolutePosition();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData leftMotor = new MotorControllerData();
        public MotorControllerData rightMotor = new MotorControllerData();
        public double heightAbsInches;
    }
}
