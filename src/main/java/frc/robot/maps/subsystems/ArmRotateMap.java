package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmRotateMap implements LoggableMap<ArmRotateMap.Data> {

    public enum ArmRotatePresets {

        OFF,

        INTAKE,

        SCOREL1,

        SCOREL23,

        SCOREL4,

        STOW,

        HOLD

    }

    public record ArmRotatePresetValues(double intake, double scoreL1, double scoreL23, double scoreL4, double stow) {
        public ArmRotatePresetValues() {
            this(0, 0, 0, 0, 0);
        }

        public double getValue(ArmRotatePresets preset) {
            switch (preset) {
                case OFF:
                    return Double.NaN;
                case INTAKE:
                    return intake;
                case SCOREL1:
                    return scoreL1;
                case SCOREL23:
                    return scoreL23;
                case SCOREL4:
                    return scoreL4;
                case STOW:
                    return stow;
                default:
                    return 0;
            }
        }
    }

    public SmartMotorController motor;
    public final IEncoder encoder;
    public final ArmRotatePresetValues armRotatePreset;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;
    public final ArmFeedforward armFeedforward;

    public ArmRotateMap(SmartMotorController motor, IEncoder encoder, ArmRotatePresetValues armRotatePreset,
            ProfiledPIDController pid, ValueRange hardLimits, ValueRange softLimits, ArmFeedforward armFeedforward) {
        this.motor = motor;
        this.encoder = encoder;
        this.armRotatePreset = armRotatePreset;
        this.pid = pid;
        this.softLimits = softLimits;
        this.hardLimits = hardLimits;
        this.armFeedforward = armFeedforward;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.rotationAbsAngleDegrees = encoder.getAbsolutePosition();
        data.rotatingAngleVelocity = encoder.getRate();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
        public double rotatingAngleVelocity;
    }
}