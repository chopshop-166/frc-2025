package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.CoralManipMap.Data;

public class CoralManip extends LoggedSubsystem<Data, CoralManipMap> {

    private final double RELEASE_SPEED = -0.3;
    private final double RELEASE_SPEED_L1 = -0.2;
    private final double INTAKE_SPEED = -0.2;
    private final double RELEASE_DELAY = 0;

    private final double RELEASE_DELAY_L1 = 0.5;
    private final double ALIGNMENT_SPEED = 0.09;
    private final double ALGAE_INTAKE = 0.4;
    private final double ALGAE_INTAKE_HAT = -0.4;

    public CoralManip(CoralManipMap coralManipMap) {
        super(new Data(), coralManipMap);
    }

    public Command scoreL1() {
        return run(() -> {
            getData().motor.setpoint = RELEASE_SPEED_L1;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command score() {
        return run(() -> {
            getData().motor.setpoint = RELEASE_SPEED;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd())
                .withName("Score Coral");
    }

    public Command betterScoreL1() {
        return run(() -> {
            getData().motor.setpoint = -RELEASE_SPEED_L1;
        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY_L1), safeStateCmd())
                .withName("Better Score L1 Coral");
    }

    public Command feed() {
        return runSafe(() -> {
            getData().motor.setpoint = RELEASE_SPEED_L1;
        }).withName("Feed Coral");
    }

    public Command feedAlgae() {
        return runOnce(() -> {
            getData().motor.setpoint = ALGAE_INTAKE;
        }).withName("Feed Algae");
    }

    public Command feedAlgaeHat() {
        return runOnce(() -> {
            getData().motor.setpoint = ALGAE_INTAKE_HAT;
        }).withName("Feed Algae Hat");
    }

    public Command intake() {
        return runSafe(() -> {
            getData().motor.setpoint = INTAKE_SPEED;
        }).until(() -> getData().gamePieceDetected).withName("Intake Coral");
    }

    public Command betterintake() {
        return run(() -> {
            getData().motor.setpoint = INTAKE_SPEED;
        }).until(() -> getData().gamePieceDetected).andThen(
                run(() -> {
                    getData().motor.setpoint = ALIGNMENT_SPEED;
                }).until(() -> !getData().gamePieceDetected),
                run(() -> {
                    getData().motor.setpoint = -ALIGNMENT_SPEED;
                }).until(() -> getData().gamePieceDetected),
                safeStateCmd()).withName("Better Intake");
    }

    // WARNING: DOES NOT REQUIRE THIS SUBSYSTEM
    public Command waitUntilGamePiece() {
        return Commands.waitUntil(() -> getData().gamePieceDetected);
    }

    @Override
    public void safeState() {
        getData().motor.setpoint = 0;
    }

    @Override
    public void reset() {
        getMap().motor.getEncoder().reset();
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
