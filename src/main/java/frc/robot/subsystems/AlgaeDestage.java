package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.AlgaeDestageMap;
import frc.robot.maps.subsystems.AlgaeDestageMap.Data;

public class AlgaeDestage extends LoggedSubsystem<Data, AlgaeDestageMap> {

    public SmartMotorController motor;

    public static double DESTAGE_SPEED = 1.0;

    public AlgaeDestage(AlgaeDestageMap algaeDestageMap) {
        super(new Data(), algaeDestageMap);
    }

    public Command destageAlgae() {
        return runSafe(() -> {
            // getData().motor.setpoint = destageSpeed;
            getData().motor.setpoint = DESTAGE_SPEED;
        });
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }
}
