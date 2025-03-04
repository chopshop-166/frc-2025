package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.FunnelMap;
import frc.robot.maps.subsystems.FunnelMap.Data;

public class Funnel extends LoggedSubsystem<Data, FunnelMap> {
    private final double RAISE_SPEED = 0.5;
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final double FUNNEL_ROTATE_SPEED = 0.2;
    private final double FUNNEL_ROTATION_FORWARD_LIMIT = 60;
    private final double FUNNEL_ROTATION_BACKWARD_LIMIT = 2;

    public Funnel(FunnelMap funnelMap) {
        super(new Data(), funnelMap);
    }

    public Command move(DoubleSupplier rotateSpeed) {

        return run(() -> {
            double speed = rotateSpeed.getAsDouble();
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                getData().motor.setpoint = (speed * speedCoef);

            } else {
                getData().motor.setpoint = 0.0;
            }

        });
    }

    public Command rotateForward() {
        return startSafe(() -> {
            getData().motor.setpoint = FUNNEL_ROTATE_SPEED;

        }).until(() -> getData().rotationDistance > FUNNEL_ROTATION_FORWARD_LIMIT);
    }

    public Command rotateBackward() {
        return startSafe(() -> {
            getData().motor.setpoint = -FUNNEL_ROTATE_SPEED;

        }).until(() -> getData().rotationDistance < FUNNEL_ROTATION_BACKWARD_LIMIT);
    }

    @Override
    public void reset() {
        getMap().encoder.reset();
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }
}
