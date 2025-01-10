package frc.robot.maps.subsystems.patterns;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class OuttakeMap implements LoggableMap<OuttakeMap.Data> {

    public final SmartMotorController leftMotor;
    public final SmartMotorController rightMotor;
    public BooleanSupplier sensor;
    public boolean splitSpeeds;

    public OuttakeMap() {
        this(new SmartMotorController(), new SmartMotorController(), true);
    }

    public OuttakeMap(SmartMotorController leftWheel, SmartMotorController rightWheel, boolean splitSpeeds) {
        this.leftMotor = leftWheel;
        this.rightMotor = rightWheel;
        this.splitSpeeds = splitSpeeds;
    }

    // Will add data/logging stuff when we need it

    @Override
    public void updateData(Data data) {
        data.leftWheel.updateData(leftMotor);
        data.rightWheel.updateData(rightMotor);
        data.gamePieceDetected = sensor.getAsBoolean();

    }

    public static class Data extends DataWrapper {
        public MotorControllerData leftWheel = new MotorControllerData();
        public MotorControllerData rightWheel = new MotorControllerData();
        public boolean gamePieceDetected;
    }

}
