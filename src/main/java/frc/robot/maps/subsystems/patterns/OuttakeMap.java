package frc.robot.maps.subsystems.patterns;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class OuttakeMap {

    public final SmartMotorController leftWheel;
    public final SmartMotorController rightWheel;
    public boolean splitSpeeds;

    public OuttakeMap() {
        this(new SmartMotorController(), new SmartMotorController(), true);
    }

    public OuttakeMap(SmartMotorController leftWheel, SmartMotorController rightWheel, boolean splitSpeeds) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
        this.splitSpeeds = splitSpeeds;
    }

    // Will add data/logging stuff when we need it

}
