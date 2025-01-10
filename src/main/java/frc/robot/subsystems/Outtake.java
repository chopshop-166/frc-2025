package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.motors.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.maps.subsystems.patterns.OuttakeMap;
import frc.robot.maps.subsystems.patterns.OuttakeMap.Data;

public class Outtake extends LoggedSubsystem<Data, OuttakeMap> {

    private final double RELEASE_SPEEDBOTH = 1;
    private final double RELEASE_SPEEDSPLIT = 0.5;
    private final double RELEASE_DELAY = 1;

    public Outtake(OuttakeMap outtakeMap) {
        super(new Data(), outtakeMap);
    }

    public Command spinOut() {
        return runSafe(() -> {
            getData().leftWheel.setpoint = RELEASE_SPEEDSPLIT + RELEASE_SPEEDBOTH;
            getData().rightWheel.setpoint = RELEASE_SPEEDBOTH;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void safeState() {
        //
    }

}
