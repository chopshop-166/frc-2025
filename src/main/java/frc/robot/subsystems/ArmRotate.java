package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.maps.subsystems.ArmRotateMap.Data;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {
    final ProfiledPIDController pid;
    private final double RAISE_SPEED_COEF = 0.5;
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final double SLOW_DOWN_COEF = 0.3;
    private final double ZEROING_SPEED = 0.5;
    private final double SAFE_ANGLE = 275.0;
    double holdAngle = 0;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    BooleanPublisher armSafePub = instance.getBooleanTopic("Arm/Safe").publish();

    DoubleSupplier armRotateSpeed;

    public ArmRotate(ArmRotateMap armRotateMap, DoubleSupplier armRotateSpeed) {
        super(new Data(), armRotateMap);
        pid = armRotateMap.pid;
        this.armRotateSpeed = armRotateSpeed;
    }

    public Command moveTo(ArmRotatePresets level) {
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(10, pid::atGoal);
        return runOnce(() -> {
            getData().preset = level;
            pid.reset(getArmAngle(), 0.0);
        }).andThen(run(() -> {
        }).until(setPointPersistenceCheck)).withName("Move To Set Angle");
    }

    public Command moveToNonOwning(ArmRotatePresets level) {
        return Commands.runOnce(() -> {
            getData().preset = level;
            pid.reset(getArmAngle(), 0.0);
        }).withName("Move To Set Angle (Non-Owning)");
    }

    public Command moveOut() {
        return runOnce(() -> {
            getData().preset = ArmRotatePresets.OUT;
            pid.reset(getArmAngle(), 0.0);
        }).andThen(run(() -> {
        }).until(() -> (getData().rotationAbsAngleDegrees <= getMap().armRotatePreset
                .applyAsDouble(ArmRotatePresets.OUT) + 2))).withName("Move To Set Angle");
    }

    public Command hold() {
        return runOnce(() -> {
            holdAngle = getArmAngle();
            getData().preset = ArmRotatePresets.HOLD;
        }).withName("Hold");
    }

    private double limits(double speed) {
        double height = getArmAngle();
        speed = getMap().hardLimits.filterSpeed(height, speed);
        speed = getMap().softLimits.scaleSpeed(height, speed, SLOW_DOWN_COEF);
        return speed;
    }

    private double getArmAngle() {
        return getData().rotationAbsAngleDegrees;
    }

    @Override
    public void periodic() {
        super.periodic();
        armSafePub.set(getData().rotationAbsAngleDegrees < SAFE_ANGLE);

        double speed = armRotateSpeed.getAsDouble();

        if (Math.abs(speed) > 0) {
            getData().preset = ArmRotatePresets.OFF;

            double speedCoef = RAISE_SPEED_COEF;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }

            getData().motor.setpoint = (limits(speed * speedCoef));

        } else if (getData().preset != ArmRotatePresets.OFF) {
            double targetHeight = getData().preset == ArmRotatePresets.HOLD ? holdAngle
                    : getMap().armRotatePreset.applyAsDouble(getData().preset);
            double setpoint = pid.calculate(getArmAngle(), new State(targetHeight, 0));
            Logger.recordOutput("ArmRotate/PID Setpoint", setpoint);

            setpoint += getMap().armFeedforward.calculate(
                    pid.getSetpoint().position,
                    pid.getSetpoint().velocity);
            Logger.recordOutput("ArmRotate/PID +FF Setpoint", setpoint);
            getData().motor.setpoint = setpoint;
        } else {
            getData().motor.setpoint = 0;
        }

        Logger.recordOutput("ArmRotate/pid at goal", pid.atGoal());
        Logger.recordOutput("ArmRotate/DesiredArmVelocity", pid.getSetpoint().velocity);
        Logger.recordOutput("ArmRotate/DesiredArmPosition", pid.getSetpoint().position);

    }

    @Override
    public void reset() {
        // Nothing to reset
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
        getData().preset = ArmRotatePresets.OFF;
    }
}