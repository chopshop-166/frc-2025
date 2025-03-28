package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.DeepClimbMap.Data;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;

public class DeepClimb extends LoggedSubsystem<Data, DeepClimbMap> {

    private final double SPOOL_IN_SPEED = 0.2;
    private final double SPOOL_OUT_SPEED = -0.72;
    final double SLOW_DOWN_COEF = 0.5;

    final double MIN_ENCODER_READING = -12;

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
            double speedCoef = 0.75;
            if (speed > 0) {
                if (getData().encoderReading >= MIN_ENCODER_READING) {
                    speedCoef = 0;
                }
            }
            getData().motor.setpoint = speed * speedCoef;
        });
    }

    public BooleanSupplier deepClimbLEDTrigger() {
        return () -> {
            return (getData().encoderReading >= MIN_ENCODER_READING);
        };
    }

    @Override
    public void reset() {
        getMap().motor.getEncoder().reset();
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }

}
