package frc.robot.maps.subsystems;

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

        INTAKE,

        SCOREL1,

        SCOREL2,

        SCOREL3,

        STOW,

        HOLD
    }

    public record ElevatorPresetValues(double intake, double scoreL1, double scoreL2, double scoreL3, double stow) {
        public ElevatorPresetValues() {
            this(0, 0, 0, 0, 0);
        }

        public double getValue(ElevatorPresets preset) {
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
                case STOW:
                    return stow;
                default:
                    return 0;
            }
        }
    }

    public final SmartMotorController leftMotor;
    public final SmartMotorController rightMotor;
    public final IEncoder encoder;
    public final ElevatorPresetValues elevatorPreset;
    public final ValueRange hardLimits;
    public final ValueRange softLimits;
    public final ProfiledPIDController pid;
    public final ElevatorFeedforward feedForward;

    public ElevatorMap() {
        this(new SmartMotorController(), new SmartMotorController(), new MockEncoder(), new ElevatorPresetValues(),
                new ValueRange(0, 0), new ValueRange(0, 0), new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)),
                new ElevatorFeedforward(0, 0, 0));
    }

    public ElevatorMap(SmartMotorController leftMotor, SmartMotorController rightMotor, IEncoder encoder,
            ElevatorPresetValues elevatorPreset, ValueRange hardLimits, ValueRange softLimits,
            ProfiledPIDController pid, ElevatorFeedforward feedForward) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.encoder = encoder;
        this.elevatorPreset = elevatorPreset;
        this.hardLimits = hardLimits;
        this.softLimits = softLimits;
        this.pid = pid;
        this.feedForward = feedForward;
    }

    @Override
    public void updateData(Data data) {
        data.leftMotor.updateData(leftMotor);
        data.rightMotor.updateData(rightMotor);
        data.heightAbsInches = encoder.getAbsolutePosition(); // need to do math to figure out the
                                                              // right value
        data.liftingHeightVelocity = encoder.getRate();
    }

    public static class Data extends DataWrapper {
        public MotorControllerData leftMotor = new MotorControllerData();
        public MotorControllerData rightMotor = new MotorControllerData();
        public double heightAbsInches;
        public double liftingHeightVelocity;
    }
}
