package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.DeepClimbMap.Data;

public class DeepClimb extends LoggedSubsystem<Data, DeepClimbMap> {

    private final double SPOOL_IN_SPEED = 1.0;
    private final double SPOOL_OUT_SPEED = -0.72;

    public DeepClimb(DeepClimbMap deepClimbMap) {
        super(new Data(), deepClimbMap);
    }

    public Command spoolIn() {
        return runSafe(() -> {
            getData().motor.setpoint = SPOOL_IN_SPEED;
        }).until(() -> getData().atBottomLimit);
    }

    public Command spoolOut() {
        return runSafe(() -> {
            getData().motor.setpoint = SPOOL_OUT_SPEED;
        });
    }

    // make the command for when the trigger is pressed spool in
    public Command spoolInJoystick(DoubleSupplier speed) {
        return runSafe(() -> {
        });
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }

}
