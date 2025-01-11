package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.patterns.AlgaeDestageMap;
import frc.robot.maps.subsystems.patterns.AlgaeDestageMap.Data;

public class AlgaeDestage extends LoggedSubsystem<Data, AlgaeDestageMap> {

    public SmartMotorController motor;

    public static double DESTAGE_SPEED = 1.0;
    public static double OFF = 0;

    public AlgaeDestage(AlgaeDestageMap algaeDestageMap) {
        super(new Data(), algaeDestageMap);
    }

    public Command destageAlgae() {
        return runSafe(() -> {
            // getData().motor.setpoint = destageSpeed;
            getData().motor.setpoint = DESTAGE_SPEED;
        });
    }

    public void safeState() {
        getData().motor.setpoint = OFF;
    }
}
