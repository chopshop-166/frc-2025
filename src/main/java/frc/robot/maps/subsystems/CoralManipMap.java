package frc.robot.maps.subsystems;

import java.util.function.BooleanSupplier;

import yams.motorcontrollers.SmartMotorController;

public record CoralManipMap(SmartMotorController motor, BooleanSupplier sensor) {
    public CoralManipMap() {
        this(null, () -> false);
    }
}
