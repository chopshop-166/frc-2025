package frc.robot.maps.subsystems;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IAbsolutePosition;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmRotateMap implements LoggableMap<ArmRotateMap.Data> {

    public enum ArmRotatePresets {

        OFF,

        INTAKE,

        SCOREL1,

        SCOREL2,

        SCOREL3,

        SCOREL4,

        OUT,

        STOW,

        HOLD

    }

    public record ArmRotatePresetValues(double intake, double scoreL1, double scoreL2, double scoreL3, double scoreL4,
            double out,
            double stow) {
        public ArmRotatePresetValues() {
            this(0, 0, 0, 0, 0, 0, 0);
        }

        public double getValue(ArmRotatePresets preset) {
            switch (preset) {
                case OFF:
                    return Double.NaN;
                case INTAKE:
                    return intake;
                case SCOREL1:
                    return scoreL1;
                case SCOREL2:
                    return scoreL2;
                case SCOREL3:
                    return scoreL3;
                case SCOREL4:
                    return scoreL4;
                case OUT:
                    return out;
                case STOW:
                    return stow;
                default:
                    return 0;
            }
        }
    }

    public SmartMotorController motor;
    public final DutyCycleEncoder encoder;
    public final ArmRotatePresetValues armRotatePreset;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;
    public final ArmFeedforward armFeedforward;

    public ArmRotateMap() {
        this(new SmartMotorController(), new DutyCycleEncoder(0), new ArmRotatePresetValues(),
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)), new ValueRange(0, 0), new ValueRange(0, 0),
                new ArmFeedforward(0, 0, 0));

    }

    public ArmRotateMap(SmartMotorController motor, DutyCycleEncoder encoder, ArmRotatePresetValues armRotatePreset,
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
        data.rotationVelocity = (data.rotationAbsAngleDegrees - encoder.get()) / .02;
        data.rotationAbsAngleDegrees = encoder.get();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
        public double rotationVelocity;
    }
}