package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.CoralManipMap.Data;

public class CoralManip extends LoggedSubsystem<Data, CoralManipMap> {

    private final double RELEASE_SPEEDRIGHT = 0.3;
    private final double RELEASE_SPEEDLEFT = 0.1;
    private final double FRONT_INTAKE_SPEED = -0.3;
    private final double REAR_INTAKE_SPEED = 0.3;
    private final double RELEASE_DELAY = 1;
    private final double DELAY = 0.0;

    private boolean isBlue = false;

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
            getData().leftMotor.setpoint = FRONT_INTAKE_SPEED;
            getData().rightMotor.setpoint = FRONT_INTAKE_SPEED;
        }).until(() -> getData().gamePieceDetected).andThen(safeStateCmd());

    }

    public Command fancyIntake() {
        return run(() -> {
            double robotAngle = 0;

            if ((isBlue && ((Math.abs(robotAngle) > 90) && (Math.abs(robotAngle) < 180)))
                    || (!isBlue && ((Math.abs(robotAngle) < 90) && (Math.abs(robotAngle) > 0)))) {
                getData().leftMotor.setpoint = FRONT_INTAKE_SPEED;
                getData().rightMotor.setpoint = FRONT_INTAKE_SPEED;

            } else {
                getData().leftMotor.setpoint = REAR_INTAKE_SPEED;
                getData().rightMotor.setpoint = REAR_INTAKE_SPEED;
            }
        }).until(() -> getData().gamePieceDetected).andThen(safeStateCmd());
    }

    @Override
    public void safeState() {
        getData().leftMotor.setpoint = 0;
        getData().rightMotor.setpoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
        isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }

}
