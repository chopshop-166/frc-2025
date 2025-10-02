package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.DeepClimbMap;
import yams.mechanisms.positional.Arm;

public class DeepClimb extends SmartSubsystemBase {

    private final double SPOOL_IN_SPEED = 0.2;
    private final double SPOOL_OUT_SPEED = -0.72;
    private final double MIN_ENCODER_READING = -12;

    final DeepClimbMap map;
    final Arm arm;
    final DeepClimbMap.Data data = new DeepClimbMap.Data();

    public DeepClimb(RobotMap robotMap) {
        map = robotMap.getDeepClimbMap(this);
        arm = new Arm(map.config);
    }

    public Command spoolIn() {
        return runSafe(() -> {
            arm.set(SPOOL_IN_SPEED);
        }).until(() -> data.atBottomLimit);
    }

    public Command spoolOut() {
        return runSafe(() -> {
            arm.set(SPOOL_OUT_SPEED);
        });
    }

    // Get joystick value to control deep climb
    public Command rotate(DoubleSupplier liftSpeed) {
        return runSafe(() -> {
            double speed = liftSpeed.getAsDouble();
            double speedCoef = 0.75;
            if (speed > 0) {
                if (data.encoderReading.magnitude() >= MIN_ENCODER_READING) {
                    speedCoef = 0;
                }
            }
            arm.set(speed * speedCoef);
        });
    }

    public BooleanSupplier deepClimbLEDTrigger() {
        return () -> {
            return (data.encoderReading.magnitude() >= MIN_ENCODER_READING);
        };
    }

    @Override
    public void periodic() {
        super.periodic();
        arm.updateTelemetry();
        map.updateData(data);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        super.simulationPeriodic();
        arm.simIterate();
    }

    @Override
    public void reset() {
        arm.setAngle(Degrees.of(0.0));
    }

    @Override
    public void safeState() {
        arm.set(0.0);
    }

}
