package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.Modifier;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
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

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoublePublisher heightPub = instance.getDoubleTopic("Elevator/Height").publish();

    ElevatorPresets level = ElevatorPresets.OFF;

    public Elevator(ElevatorMap elevatorMap) {
        super(new Data(), elevatorMap);
        pid = elevatorMap.pid;
    }

    public Command move(DoubleSupplier liftSpeed) {

        return run(() -> {
            double speed = liftSpeed.getAsDouble();
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                level = ElevatorPresets.OFF;
                getData().motor.setpoint = limits(speed * speedCoef);
            } else if (level == ElevatorPresets.OFF) {
                getData().motor.setpoint = 0.0;
            }

        });
    }

    public Command moveToZero() {
        return startSafe(() -> {
            getData().motor.setpoint = LOWER_SPEED;
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
        getData().motor.setpoint = 0;
        level = ElevatorPresets.OFF;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (level != ElevatorPresets.OFF) {
            double targetHeight = level == ElevatorPresets.HOLD ? holdHeight : getMap().elevatorPreset.getValue(level);
            double setpoint = pid.calculate(getElevatorHeight(), new State(targetHeight, 0));
            setpoint += getMap().feedForward.calculate(
                    pid.getSetpoint().position,
                    pid.getSetpoint().velocity);
            getData().motor.setpoint = setpoint;
        }

        Logger.recordOutput("Elevator preset", level);
        Logger.recordOutput("DesiredElevatorVelocity", pid.getSetpoint().velocity);
        Logger.recordOutput("DesiredElevatorPosition", pid.getSetpoint().position);

    }

}
