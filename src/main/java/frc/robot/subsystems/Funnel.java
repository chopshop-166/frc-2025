package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.FunnelMap;
import frc.robot.maps.subsystems.FunnelMap.Data;

public class Funnel extends LoggedSubsystem<Data, FunnelMap> {
    private final double RAISE_SPEED = 0.5;
    private final double LOWER_SPEED = 0.3;
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final double SLOW_DOWN_COEF = 0.3;

    public Funnel(FunnelMap funnelMap) {
        super(new Data(), funnelMap);
    }

    Command move(DoubleSupplier rotateSpeed) {

        return run(() -> {
            double speed = rotateSpeed.getAsDouble();
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                getData().motor.setpoint = (limits(speed * speedCoef));

            } else {
                getData().motor.setpoint = 0.0;
            }

        });
    }

    private double limits(double speed) {
        double height = getFunnelAngle();
        speed = getMap().hardLimits.filterSpeed(height, speed);
        speed = getMap().softLimits.scaleSpeed(height, speed, SLOW_DOWN_COEF);
        return speed;
    }

    private double getFunnelAngle() {
        return getData().rotationAbsAngleDegrees;
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
