package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.maps.subsystems.patterns.OuttakeMap;
import frc.robot.maps.subsystems.patterns.OuttakeMap.Data;

public class Outtake extends LoggedSubsystem<Data, OuttakeMap> {

    private final double RELEASE_SPEEDRIGHT = 1;
    private final double RELEASE_SPEEDLEFT = 0.5;
    private final double INTAKE_SPEED = 0.3;
    private final double RELEASE_DELAY = 1;
    private final double DELAY = 0.5;
    private final double OFF = 0;

    public Outtake(OuttakeMap outtakeMap) {
        super(new Data(), outtakeMap);
    }

    public Command spinOut() {
        return run(() -> {
            getData().leftWheel.setpoint = RELEASE_SPEEDLEFT;
            getData().rightWheel.setpoint = RELEASE_SPEEDRIGHT;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command spinIn() {
        return run(() -> {
            getData().leftWheel.setpoint = INTAKE_SPEED;
            getData().rightWheel.setpoint = INTAKE_SPEED;
        }).until(() -> getData().gamePieceDetected).andThen(waitSeconds(DELAY), safeStateCmd());
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void safeState() {
        getData().leftWheel.setpoint = OFF;
        getData().rightWheel.setpoint = OFF;
        //
    }

}
