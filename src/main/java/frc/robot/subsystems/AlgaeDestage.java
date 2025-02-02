package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.AlgaeDestageMap;
import frc.robot.maps.subsystems.AlgaeDestageMap.Data;

public class AlgaeDestage extends LoggedSubsystem<Data, AlgaeDestageMap> {

    public static double DESTAGE_SPEED = -0.5;

    public AlgaeDestage(AlgaeDestageMap algaeDestageMap) {
        super(new Data(), algaeDestageMap);
    }

    public Command destageAlgae() {
        return runSafe(() -> {
            getData().motor.setpoint = DESTAGE_SPEED;
        });
    }

    @Override
    public void reset() {
        safeState();
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }
}
