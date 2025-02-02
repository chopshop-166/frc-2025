package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.CoralManipMap.Data;

public class CoralManip extends LoggedSubsystem<Data, CoralManipMap> {

    private final double RELEASE_SPEEDRIGHT = 0.3;
    private final double RELEASE_SPEEDLEFT = 0.1;
    private final double INTAKE_SPEED = 0.3;
    private final double RELEASE_DELAY = 1;
    private final double DELAY = 0.0;

    public CoralManip(CoralManipMap coralManipMap) {
        super(new Data(), coralManipMap);
    }

    public Command scoreL1() {
        return run(() -> {
            getData().leftMotor.setpoint = RELEASE_SPEEDLEFT;
            getData().rightMotor.setpoint = RELEASE_SPEEDRIGHT;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command score() {
        return run(() -> {
            getData().leftMotor.setpoint = RELEASE_SPEEDRIGHT;
            getData().rightMotor.setpoint = RELEASE_SPEEDRIGHT;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command intake() {
        return run(() -> {
            getData().leftMotor.setpoint = INTAKE_SPEED;
            getData().rightMotor.setpoint = INTAKE_SPEED;
        }).until(() -> getData().gamePieceDetected).andThen(safeStateCmd());
    }

    @Override
    public void safeState() {
        getData().leftMotor.setpoint = 0;
        getData().rightMotor.setpoint = 0;
    }

}
