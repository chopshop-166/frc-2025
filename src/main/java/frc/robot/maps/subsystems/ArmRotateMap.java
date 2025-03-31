package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmRotateMap implements LoggableMap<ArmRotateMap.Data> {

    public enum ArmRotatePresets {

        OFF,

        INTAKE,

        SCOREL1,

        SCOREL1_TAKETWO,

        SCOREL2,

        SCOREL3,

        SCOREL4,

        SCOREL4_AUTO,

        OUT,

        STOW,

        ALGAE,

        HOLD

    }

    public interface PresetValue extends ToDoubleFunction<ArmRotatePresets> {
    }

    public SmartMotorController motor;
    public final DutyCycleEncoder encoder;
    public final PresetValue armRotatePreset;
    public final ProfiledPIDController pid;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;
    public final ArmFeedforward armFeedforward;

    public ArmRotateMap() {
        this(new SmartMotorController(), new DutyCycleEncoder(0), p -> Double.NaN,
                new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)), new ValueRange(0, 0), new ValueRange(0, 0),
                new ArmFeedforward(0, 0, 0));

    }

    public ArmRotateMap(SmartMotorController motor, DutyCycleEncoder encoder, PresetValue armRotatePreset,
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
        data.rotationAbsAngleDegrees = encoder.get();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double rotationAbsAngleDegrees;
        public ArmRotatePresets preset = ArmRotatePresets.OFF;
    }
}