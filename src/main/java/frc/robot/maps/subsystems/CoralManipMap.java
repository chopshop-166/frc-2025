package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class CoralManipMap implements LoggableMap<CoralManipMap.Data> {
    public final SmartMotorController leftMotor;
    public final SmartMotorController rightMotor;
    public final BooleanSupplier sensor;

    public CoralManipMap() {
        this(new SmartMotorController(), new SmartMotorController(), () -> false);
    }

    public CoralManipMap(SmartMotorController leftMotor, SmartMotorController rightMotor, BooleanSupplier sensor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.sensor = sensor;
    }

    @Override
    public void updateData(Data data) {
        data.leftMotor.updateData(leftMotor);
        data.rightMotor.updateData(rightMotor);
        data.gamePieceDetected = sensor.getAsBoolean();

    }

    public static class Data extends DataWrapper {
        public MotorControllerData leftMotor = new MotorControllerData();
        public MotorControllerData rightMotor = new MotorControllerData();
        public boolean gamePieceDetected;
    }

}
