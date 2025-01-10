package frc.robot.maps.subsystems.patterns;

import com.chopshop166.chopshoplib.motors.SmartMotorController;

public class AlgaeDestageMap {

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

}
