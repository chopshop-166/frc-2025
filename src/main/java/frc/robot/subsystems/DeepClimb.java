package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.DeepClimbMap.Data;

public class DeepClimb extends LoggedSubsystem<Data, DeepClimbMap> {

    public DeepClimb(DeepClimbMap deepClimbMap) {
        super(new Data(), deepClimbMap);
    }

    public Command spoolIn() {
        return runSafe(() -> {
            getData().motor.setpoint = 1;
        }).until(() -> getData().atTopLimit).andThen(safeStateCmd());
    }

    public Command spoolOut() {
        return runSafe(() -> {
            getData().motor.setpoint = -1;
        }).until(() -> getData().atBottomLimit).andThen(safeStateCmd());
    }

    @Override
    public void safeState() {
        // no safestate to set
    }

}
