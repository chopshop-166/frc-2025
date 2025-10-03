package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import yams.mechanisms.positional.Arm;

public class ArmRotate extends SmartSubsystemBase {
    private final double RAISE_SPEED_COEF = 0.5;
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final Angle SAFE_ANGLE = Degrees.of(275.0);
    double holdAngle = 0;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    BooleanPublisher armSafePub = instance.getBooleanTopic("Arm/Safe").publish();

    ArmRotateMap map;
    Arm arm;
    private final ArmRotateMap.Data data = new ArmRotateMap.Data();

    public ArmRotate(RobotMap robotMap) {
        map = robotMap.getArmRotateConfig(this);
        arm = new Arm(map.config);
    }

    public Command moveTo(ArmRotatePresets level) {
        return arm.set(() -> map.armRotatePreset.applyAsDouble(level));
    }

    public Command moveOut() {
        return arm.set(() -> map.armRotatePreset.applyAsDouble(ArmRotatePresets.OUT))
                .andThen(run(() -> {
                }).until(arm.lte(Degrees.of(map.armRotatePreset.applyAsDouble(ArmRotatePresets.OUT) + 2))))
                .withName("Move Out");
    }

    public Command move(DoubleSupplier armRotateSpeed) {
        return arm.set(() -> {
            double speed = armRotateSpeed.getAsDouble();

            double speedCoef = RAISE_SPEED_COEF;
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }

            return speed * speedCoef;
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        arm.updateTelemetry();
        armSafePub.set(arm.getAngle().lt(SAFE_ANGLE));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        super.simulationPeriodic();
        arm.simIterate();
        map.updateData(data);
    }

    @Override
    public void reset() {
        // Nothing to reset
    }

    @Override
    public void safeState() {
        arm.set(0.0).schedule();
    }
}