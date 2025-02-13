package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {
    final ProfiledPIDController pid;
    private final double RAISE_SPEED = 0.5;
    private final double LOWER_SPEED = 0.3;
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final double SLOW_DOWN_COEF = 0.3;
    private final double ZEROING_SPEED = 0.5;
    double holdAngle = 0;

    ArmRotatePresets preset = ArmRotatePresets.OFF;

    public ArmRotate(ArmRotateMap armRotateMap) {
        super(new Data(), armRotateMap);
        pid = armRotateMap.pid;
    }

    public Command move(DoubleSupplier rotateSpeed) {

        return run(() -> {
            double speed = rotateSpeed.getAsDouble();
            double speedCoef = RAISE_SPEED;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                preset = ArmRotatePresets.OFF;
                getData().motor.setpoint = (limits(speed * speedCoef));
            } else if (preset == ArmRotatePresets.OFF) {
                getData().motor.setpoint = 0.0;
            } else {
                getData().motor.setpoint = 0.0;
            }

        });
    }

    public Command moveTo(ArmRotatePresets level) {
        PersistenceCheck setPointPersistenceCheck = new PersistenceCheck(30, pid::atGoal);
        return runOnce(() -> {
            pid.reset(getArmAngle(), getData().rotatingAngleVelocity);
        }).andThen(run(() -> {
            Logger.recordOutput("Pid at goal", pid.atGoal());
        }).until(setPointPersistenceCheck)).withName("Move To Set Angle");
    }

    public Command zero() {
        return runSafe(() -> {
            getData().motor.setpoint = ZEROING_SPEED;
        }).until(() -> getMap().motor.validate()).andThen(resetCmd());
    }

    public Command moveToZero() {
        return startSafe(() -> {
            getData().motor.setpoint = ZEROING_SPEED;
            preset = ArmRotatePresets.OFF;
        }).until(() -> {
            return getArmAngle() > getMap().armRotatePreset.getValue(ArmRotatePresets.STOW);
        });
    }

    public Command hold() {
        return runOnce(() -> {
            holdAngle = getArmAngle();
            this.preset = ArmRotatePresets.HOLD;
        });
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
        if (preset != ArmRotatePresets.OFF) {
            double targetHeight = preset == ArmRotatePresets.HOLD ? holdAngle
                    : getMap().armRotatePreset.getValue(preset);
            double setpoint = pid.calculate(getArmAngle(), new State(targetHeight, 0));
            setpoint += getMap().armFeedforward.calculate(
                    pid.getSetpoint().position,
                    pid.getSetpoint().velocity);
            getData().motor.setpoint = setpoint;
        }

        Logger.recordOutput("Arm preset", preset);
        Logger.recordOutput("DesiredArmVelocity", pid.getSetpoint().velocity);
        Logger.recordOutput("DesiredArmPosition", pid.getSetpoint().position);

    }

    @Override
    public void reset() {
        getMap().encoder.reset();
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
        preset = ArmRotatePresets.OFF;
    }
}