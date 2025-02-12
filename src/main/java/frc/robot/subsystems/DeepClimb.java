package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.DeepClimbMap.Data;

public class DeepClimb extends LoggedSubsystem<Data, DeepClimbMap> {

    private final double SPOOL_IN_SPEED = 1.0;
    private final double SPOOL_OUT_SPEED = -0.72;
    final double RAISE_SPEED_COEF = 1.0;
    final double MANUAL_LOWER_SPEED_COEF = 1.0;
    final double SLOW_DOWN_COEF = 0.5;

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

    // Get joystick value to control deep climb
    public Command rotate(DoubleSupplier liftSpeed) {
        return runSafe(() -> {
            double speed = liftSpeed.getAsDouble();
            double speedCoef = RAISE_SPEED_COEF;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }

            getData().motor.setpoint = speed * speedCoef;

        });
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }

}
