package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.OuttakeMap;
import frc.robot.maps.subsystems.OuttakeMap.Data;

public class Outtake extends LoggedSubsystem<Data, OuttakeMap> {

    private final double RELEASE_SPEEDRIGHT = 0.3;
    private final double RELEASE_SPEEDLEFT = 0.1;
    private final double INTAKE_SPEED = 0.3;
    private final double RELEASE_DELAY = 1;
    private final double REVERSE = -0.5;
    private final double DELAY = 0.0;

    public Outtake(OuttakeMap outtakeMap) {
        super(new Data(), outtakeMap);
    }

    public Command scoreL1() {
        return run(() -> {
            getData().leftWheel.setpoint = RELEASE_SPEEDLEFT;
            getData().rightWheel.setpoint = RELEASE_SPEEDRIGHT;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command score() {
        return run(() -> {
            getData().leftWheel.setpoint = RELEASE_SPEEDRIGHT;
            getData().rightWheel.setpoint = RELEASE_SPEEDRIGHT;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command intake() {
        return run(() -> {
            getData().leftWheel.setpoint = INTAKE_SPEED;
            getData().rightWheel.setpoint = INTAKE_SPEED;
        }).until(() -> getData().gamePieceDetected).andThen(safeStateCmd());
    }

    public Command reverse() {
        return runSafe(() -> {
            getData().leftWheel.setpoint = REVERSE;
            getData().rightWheel.setpoint = REVERSE;
        });
    }

    @Override
    public void reset() {
        safeState();
    }

    @Override
    public void safeState() {
        getData().leftWheel.setpoint = 0;
        getData().rightWheel.setpoint = 0;
        //
    }

}
