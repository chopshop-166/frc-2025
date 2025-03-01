package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.ElevatorMap.Data;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;

public class Elevator extends LoggedSubsystem<Data, ElevatorMap> {

    final ProfiledPIDController pid;
    final double SLOW_DOWN_COEF = 0.5;
    final double LOWER_SPEED = -0.15;
    final double ZEROING_SPEED = -0.1;
    double holdHeight = 0;

    final static double BAD_HEIGHT_LOWER = 12;
    final static double BAD_HEIGHT_UPPER = 15;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoublePublisher heightPub = instance.getDoubleTopic("Elevator/Height").publish();
    BooleanSubscriber armSafeSub = instance.getBooleanTopic("Arm/Safe").subscribe(false);

    DoubleSupplier elevatorSpeed;
    ElevatorPresets level = ElevatorPresets.OFF;

    public Elevator(ElevatorMap elevatorMap, DoubleSupplier elevatorSpeed) {
        super(new Data(), elevatorMap);
        pid = elevatorMap.pid;
        this.elevatorSpeed = elevatorSpeed;
    }

    public Command move(DoubleSupplier liftSpeed) {
        return run(() -> {
            double speed = liftSpeed.getAsDouble();
            if (armSafeSub.getAsBoolean()
                    || (getElevatorHeight() < BAD_HEIGHT_LOWER && speed < 0)
                    || (getElevatorHeight() > BAD_HEIGHT_UPPER && speed > 0)) {
                if (Math.abs(speed) > 0) {
                    level = ElevatorPresets.OFF;
                    getData().motor.setpoint = limits(speed);
                } else if (level == ElevatorPresets.OFF) {
                    getData().motor.setpoint = 0.0;
                }
            } else if (level == ElevatorPresets.OFF) {
                getData().motor.setpoint = 0.0;
            }
        });
    }

    public Command zero() {
        return startSafe(() -> {
            getMap().motor.resetValidators();
            level = ElevatorPresets.ZEROING;
            getData().motor.setpoint = ZEROING_SPEED;

        }).until(() -> getMap().motor.validate()).andThen(resetCmd());
    }

    public Command moveTo(ElevatorPresets level) {
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(30, pid::atGoal);
        return runOnce(() -> {
            this.level = level;
            pid.reset(getElevatorHeight(), getData().liftingHeightVelocity);
        }).andThen(run(() -> {
            Logger.recordOutput("PID at goal", pid.atGoal());
        })).until(() -> {
            return setPointPersistenceCheck.getAsBoolean();
        }).withName("Move to set height");
    }

    public BooleanSupplier elevatorSafeTrigger() {
        return () -> {
            return (getData().heightAbsInches < BAD_HEIGHT_LOWER && level == ElevatorPresets.INTAKE);
        };
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
        level = ElevatorPresets.OFF;
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
        heightPub.set(getData().heightAbsInches / getMap().hardLimits.max());
        double speed = elevatorSpeed.getAsDouble();

        if (Math.abs(speed) > 0) {
            level = ElevatorPresets.OFF;

            if (armSafeSub.getAsBoolean()
                    || (getElevatorHeight() < BAD_HEIGHT_LOWER && speed < 0)
                    || (getElevatorHeight() > BAD_HEIGHT_UPPER && speed > 0)) {
                getData().motor.setpoint = limits(speed);
            }
        } else if (level == ElevatorPresets.ZEROING) {
            // Do nothing
        } else if (level != ElevatorPresets.OFF) {
            double targetHeight = level == ElevatorPresets.HOLD ? holdHeight
                    : getMap().presetValues.applyAsDouble(level);
            double setpoint = pid.calculate(getElevatorHeight(), new State(targetHeight, 0));
            setpoint += getMap().feedForward.calculate(
                    pid.getSetpoint().position,
                    pid.getSetpoint().velocity);
            getData().motor.setpoint = setpoint;
        } else {
            getData().motor.setpoint = 0;
        }

        Logger.recordOutput("Elevator preset", level);
        Logger.recordOutput("DesiredElevatorVelocity", pid.getSetpoint().velocity);
        Logger.recordOutput("DesiredElevatorPosition", pid.getSetpoint().position);

    }
}