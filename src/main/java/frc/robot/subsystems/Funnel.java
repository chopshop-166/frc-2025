package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.FunnelMap;
import yams.mechanisms.positional.Arm;

public class Funnel extends SmartSubsystemBase {
    private final double MANUAL_LOWER_SPEED_COEF = 0.3;
    private final double FUNNEL_ROTATE_SPEED = 0.2;

    private final FunnelMap map;
    private final Arm arm;

    public Funnel(RobotMap robotMap) {
        map = robotMap.getFunnelMap(this);
        arm = new Arm(map.armConfig());
    }

    public Command move(DoubleSupplier rotateSpeed) {
        return arm.set(() -> {
            double speed = rotateSpeed.getAsDouble();
            double speedCoef = map.driveSpeed();
            if (speed < 0) {
                speedCoef = MANUAL_LOWER_SPEED_COEF;
            }
            if (Math.abs(speed) > 0) {
                return speed * speedCoef;
            } else {
                return 0.0;
            }
        });
    }

    public Command rotateForward() {
        return arm.set(FUNNEL_ROTATE_SPEED);
    }

    public Command rotateBackward() {
        return arm.set(-FUNNEL_ROTATE_SPEED);
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
        arm.set(0.0);
    }
}
