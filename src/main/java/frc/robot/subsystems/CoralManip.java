package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.CoralManipMap.Data;

public class CoralManip extends LoggedSubsystem<Data, CoralManipMap> {

    private final double RELEASE_SPEEDRIGHT = -0.3;
    private final double RELEASE_SPEEDLEFT = -0.1;
    private final double INTAKE_SPEED = -0.3;
    private final double RELEASE_DELAY = 1;
    private final double ALIGNMENT_DISTANCE = 0.0;
    private final double ALIGNMENT_SPEED = 0.1;

    public CoralManip(CoralManipMap coralManipMap) {
        super(new Data(), coralManipMap);
    }

    public Command scoreL1() {
        return run(() -> {
            getData().motor.setpoint = RELEASE_SPEEDLEFT;

        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command scoreL4() {
        return run(() -> {
            getData().motor.setpoint = RELEASE_SPEEDLEFT;

        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command score() {
        return run(() -> {
            getData().motor.setpoint = RELEASE_SPEEDRIGHT;

        }).until(() -> !getData().gamePieceDetected).andThen(waitSeconds(RELEASE_DELAY), safeStateCmd());
    }

    public Command feed() {
        return runSafe(() -> {
            getData().motor.setpoint = RELEASE_SPEEDLEFT;

        });
    }

    public Command feedBack() {
        return runSafe(() -> {
            getData().motor.setpoint = -RELEASE_SPEEDLEFT;

        });
    }

    public Command intake() {
        return runSafe(() -> {
            getData().motor.setpoint = INTAKE_SPEED;

        }).until(() -> getData().gamePieceDetected);
    }

    public Command betterintake() {
        return sequence(
                runOnce(() -> {
                    getData().motor.setpoint = INTAKE_SPEED;
                }),
                waitUntil(() -> getData().gamePieceDetected),
                runOnce(() -> {
                    getData().motor.setpoint = ALIGNMENT_SPEED;
                }),
                waitUntil(() -> !getData().gamePieceDetected),
                runOnce(() -> {
                    getData().motor.setpoint = -ALIGNMENT_SPEED;
                }),
                waitUntil(() -> getData().gamePieceDetected),
                safeStateCmd());
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
        Logger.recordOutput("Intake encoder distance", getMap().motor.getEncoder().getDistance());
        Logger.recordOutput("Game piece detected", getData().gamePieceDetected);
    }
}
