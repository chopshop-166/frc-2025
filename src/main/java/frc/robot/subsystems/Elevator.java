package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.ElevatorMap.Data;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;

public class Elevator extends LoggedSubsystem<Data, ElevatorMap> {

    final ProfiledPIDController pid;
    final double RAISE_SPEED = .85;
    final double MANUAL_LOWER_SPEED_COEF = 0.5;
    final double SLOW_DOWN_COEF = 0.5;
    final double LOWER_SPEED = -0.15;
    double holdHeight = 0;

    ElevatorPresets level = ElevatorPresets.OFF;

    public Elevator(ElevatorMap elevatorMap) {
        super(new Data(), elevatorMap);
        pid = elevatorMap.pid;
    }

    public Command move(DoubleSupplier liftSpeed) {
        Modifier deadband = Modifier.deadband(.1);

        return run(() -> {
            double speed = deadband.applyAsDouble(liftSpeed.getAsDouble());
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                level = ElevatorPresets.OFF;
                getData().setSetpoint(limits(speed * speedCoef));
            } else if (level == ElevatorPresets.OFF) {
                getData().setSetpoint(0.0);
            }

        });
    }

    public Command moveToZero() {
        return startSafe(() -> {
            getData().setSetpoint(LOWER_SPEED);
            level = ElevatorPresets.OFF;
        }).until(() -> {
            return getElevatorHeight() < getMap().elevatorPreset.getValue(ElevatorPresets.STOW);
        });
    }

    public Command moveTo(ElevatorPresets level) {
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(30, pid::atGoal);
        return runOnce(() -> {
            this.level = level;
            pid.reset(getElevatorHeight(), getData().liftingHeightVelocity);
        }).andThen(run(() -> {
            Logger.recordOutput("PID at goal", pid.atGoal());
        })).until(setPointPersistenceCheck).withName("Move to set height");
    }

    public Command hold() {
        return runOnce(() -> {
            holdHeight = getElevatorHeight();
            this.level = ElevatorPresets.HOLD;
        });
    }

    private double limits(double speed) {
        double height = getElevatorHeight();

        speed = getMap().hardLimits.filterSpeed(height, speed);

        speed = getMap().softLimits.scaleSpeed(height, speed, SLOW_DOWN_COEF);

        return speed;

    }

    private double getElevatorHeight() {
        return getData().heightAbsInches;
    }

    @Override
    public void reset() {
        getMap().encoder.reset();
    }

    @Override
    public void safeState() {
        getData().setSetpoint(0);
    }

}
