package frc.robot.maps.subsystems;

import java.util.function.ToDoubleFunction;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.MotorControllerData;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorMap implements LoggableMap<ElevatorMap.Data> {

    public enum ElevatorPresets {
        OFF,

        ZEROING,

        INTAKE,

        SCOREL1,

        SCOREL1_TAKETWO,

        SCOREL2,

        ALGAEL2,

        SCOREL3,

        ALGAEL3,

        SCOREL4,

        HIGHESTPOINT,

        BEFOREALGAE,

        STOW,

        HOLD
    }

    public interface PresetValues extends ToDoubleFunction<ElevatorPresets> {
    }

    public final SmartMotorController motor;
    public final IEncoder encoder;
    public final PresetValues presetValues;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;
    public final ProfiledPIDController pid;
    public final ElevatorFeedforward feedForward;

    public ElevatorMap() {
        this(new SmartMotorController(), new MockEncoder(), p -> Double.NaN,
                new ValueRange(0, 0), new ValueRange(0, 0), new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
                new ElevatorFeedforward(0, 0, 0));
    }

    public ElevatorMap(SmartMotorController motor, IEncoder encoder,
            PresetValues presetValues, ValueRange hardLimits, ValueRange softLimits,
            ProfiledPIDController pid, ElevatorFeedforward feedForward) {
        this.motor = motor;
        this.encoder = encoder;
        this.presetValues = presetValues;
        this.hardLimits = hardLimits;
        this.softLimits = softLimits;
        this.pid = pid;
        this.feedForward = feedForward;
    }

    @Override
    public void updateData(Data data) {
        data.motor.updateData(motor);
        data.heightAbsInches = encoder.getDistance(); // need to do math to figure out the
                                                      // right value
        data.liftingHeightVelocity = encoder.getRate();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData motor = new MotorControllerData();
        public double heightAbsInches;
        public double liftingHeightVelocity;
        public ElevatorPresets preset = ElevatorPresets.OFF;
    }
}
