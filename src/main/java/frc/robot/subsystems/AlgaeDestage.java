package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.patterns.AlgaeDestageMap;
import frc.robot.maps.subsystems.patterns.AlgaeDestageMap.Data;

public class AlgaeDestage extends LoggedSubsystem<Data, AlgaeDestageMap> {

    public SmartMotorController motor;

    public double destageSpeed = 1.0;

    public AlgaeDestage(AlgaeDestageMap algaeDestageMap) {
        super(new Data(), algaeDestageMap);
    }

    public AlgaeDestage() {
        this(new SmartMotorController());

    }

    public AlgaeDestage(SmartMotorController motor) {
        this.motor = motor;

    }

    public Command destageAlgae() {
        return new Command(() -> {
            getData().motor.setpoint = destageSpeed;
        });
    }
}
