package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.CoralManipMap;

public class CoralManip extends SmartSubsystemBase {

    private final double RELEASE_SPEED = -0.3;
    private final double RELEASE_SPEED_L1 = -0.2;
    private final double INTAKE_SPEED = -0.2;
    private final double RELEASE_DELAY = 0;

    private final double RELEASE_DELAY_L1 = 0.5;
    private final double ALIGNMENT_SPEED = 0.09;
    private final double ALGAE_INTAKE = 0.4;
    private final double ALGAE_INTAKE_HAT = -0.4;

    private final CoralManipMap map;
    private boolean gamePieceDetected;

    public CoralManip(RobotMap robotMap) {
        map = robotMap.getCoralManipMap(this);
    }

    public Command scoreL1() {
        return run(() -> {
            map.motor().setDutyCycle(RELEASE_SPEED_L1);
        }).until(() -> !gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command score() {
        return run(() -> {
            map.motor().setDutyCycle(RELEASE_SPEED);
        }).until(() -> !gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd())
                .withName("Score Coral");
    }

    public Command betterScoreL1() {
        return run(() -> {
            map.motor().setDutyCycle(-RELEASE_SPEED_L1);
        }).until(() -> !gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY_L1), safeStateCmd())
                .withName("Better Score L1 Coral");
    }

    public Command feed() {
        return runSafe(() -> {
            map.motor().setDutyCycle(RELEASE_SPEED_L1);
        }).withName("Feed Coral");
    }

    public Command feedAlgae() {
        return runOnce(() -> {
            map.motor().setDutyCycle(ALGAE_INTAKE);
        }).withName("Feed Algae");
    }

    public Command feedAlgaeHat() {
        return runOnce(() -> {
            map.motor().setDutyCycle(ALGAE_INTAKE_HAT);
        }).withName("Feed Algae Hat");
    }

    public Command intake() {
        return runSafe(() -> {
            map.motor().setDutyCycle(INTAKE_SPEED);
        }).until(() -> gamePieceDetected).withName("Intake Coral");
    }

    public Command betterintake() {
        return Commands.sequence(
                run(() -> {
                    map.motor().setDutyCycle(INTAKE_SPEED);
                }).until(() -> gamePieceDetected),
                run(() -> {
                    map.motor().setDutyCycle(ALIGNMENT_SPEED);
                }).until(() -> !gamePieceDetected),
                run(() -> {
                    map.motor().setDutyCycle(-ALIGNMENT_SPEED);
                }).until(() -> gamePieceDetected),
                safeStateCmd())
                .withName("Better Intake");
    }

    @Override
    public void safeState() {
        map.motor().setDutyCycle(0);
    }

    @Override
    public void reset() {
    }

    @Override
    public void periodic() {
        super.periodic();
        map.motor().updateTelemetry();
        gamePieceDetected = map.sensor().getAsBoolean();
        Logger.recordOutput("Coral Manip/Game Piece Detected", gamePieceDetected);
    }
}
