package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;

public class Elevator extends SmartSubsystemBase {

    private final ElevatorMap map;
    private final yams.mechanisms.positional.Elevator elevator;

    final double ZEROING_SPEED = -0.1;

    final static Distance BAD_HEIGHT_LOWER = Inches.of(12);

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoublePublisher heightPub = instance.getDoubleTopic("Elevator/Height").publish();
    BooleanSubscriber armSafeSub = instance.getBooleanTopic("Arm/Safe").subscribe(false);

    public Elevator(RobotMap robotMap) {
        map = robotMap.getElevatorMap(this);
        this.elevator = map.elevator;
    }

    public Command zero() {
        var debouncer = new Debouncer(0.2);
        return elevator.set(ZEROING_SPEED).andThen(run(() -> {
        }).until(() -> {
            return debouncer.calculate(elevator.getVelocity().lt(InchesPerSecond.of(10)));
        })).andThen(resetCmd());
    }

    public Command moveTo(ElevatorPresets level) {
        return elevator.setHeight(Inches.of(map.presetValues.applyAsDouble(level)));
    }

    public Trigger elevatorSafeTrigger() {
        return elevator.lte(BAD_HEIGHT_LOWER);
    }

    public Command move(DoubleSupplier elevatorSpeed) {
        return elevator.set(elevatorSpeed::getAsDouble);
    }

    public boolean atPreset(ElevatorPresets preset) {
        return elevator.isNear(Inches.of(map.presetValues.applyAsDouble(preset)), Inches.of(1)).getAsBoolean();
    }

    @Override
    public void reset() {
    }

    @Override
    public void safeState() {
        elevator.set(0.0).schedule();
    }

    @Override
    public void periodic() {
        super.periodic();
        elevator.updateTelemetry();
        heightPub.set(elevator.getHeight().in(Inches));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        super.simulationPeriodic();
        elevator.simIterate();
    }
}