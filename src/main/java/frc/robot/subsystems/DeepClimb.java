package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.maps.RobotMap;
import yams.mechanisms.positional.Arm;

public class DeepClimb extends SmartSubsystemBase {

    private final double SPOOL_IN_SPEED = 0.2;
    private final double SPOOL_OUT_SPEED = -0.72;
    private final double MIN_ENCODER_READING = -12;

    private final Arm arm;

    public DeepClimb(RobotMap robotMap) {
        arm = new Arm(robotMap.getDeepClimbConfig(this));
    }

    public Command spoolIn() {
        return arm.set(SPOOL_IN_SPEED).finallyDo(this::safeState);
    }

    public Command spoolOut() {
        return arm.set(SPOOL_OUT_SPEED).finallyDo(this::safeState);
    }

    // Get joystick value to control deep climb
    public Command rotate(DoubleSupplier liftSpeed) {
        return arm.set(() -> {
            double speed = liftSpeed.getAsDouble();
            double speedCoef = 0.75;
            if (speed > 0) {
                if (deepClimbLEDTrigger().getAsBoolean()) {
                    speedCoef = 0;
                }
            }
            return speed * speedCoef;
        });
    }

    public Trigger deepClimbLEDTrigger() {
        return arm.gte(Degrees.of(MIN_ENCODER_READING));
    }

    @Override
    public void periodic() {
        super.periodic();
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        super.simulationPeriodic();
        arm.simIterate();
    }

    @Override
    public void reset() {
    }

    @Override
    public void safeState() {
        arm.set(0.0).schedule();
    }

}
