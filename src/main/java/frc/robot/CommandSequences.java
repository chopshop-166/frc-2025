package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.CoralManip;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Led;

public class CommandSequences {

    Drive drive;
    Led led;
    CoralManip coralManip;
    Elevator elevator;
    ArmRotate armRotate;
    Funnel funnel;
    DeepClimb deepClimb;

    public CommandSequences(Drive drive, Led led, CoralManip coralManip, Elevator elevator,
            ArmRotate armRotate, Funnel funnel, DeepClimb deepClimb) {
        this.drive = drive;
        this.led = led;
        this.coralManip = coralManip;
        this.elevator = elevator;
        this.armRotate = armRotate;
        this.funnel = funnel;
        this.deepClimb = deepClimb;
    }

    // Moves elevator to intake preset and intakes when preset has been reached
    // Intakes until sensor is tripped, LEDs indicate that game piece is acquired

    public Command intake() {
        return Commands.either(Commands.none(), armOutLED().andThen(led.elevatorToPreset()),
                () -> elevator.atPreset(ElevatorPresets.INTAKE) || elevator.atPreset(ElevatorPresets.STOW))
                .andThen(elevator.clearPreset(),
                        elevator.moveTo(ElevatorPresets.INTAKE))
                .withName("Intake");
    }

    public Command intakeBottom() {
        return sequence(armRotate.moveTo(ArmRotatePresets.INTAKE), led.armAtPreset())
                .alongWith(
                        intakeWithLEDs())
                .andThen(led.colorAlliance())
                .withName("Intake Bottom");
    }

    public Command intakeWithLEDs() {
        return parallel(coralManip.betterintake(), led.intakingStingray());
    }
    // Moves elevator to set coral preset

    public Command moveElevator(ElevatorPresets level, ArmRotatePresets preset) {
        return armOutLED().andThen(led.elevatorToPreset(), elevator.moveTo(level),
                moveArm(preset), led.colorAlliance()).withName("Move Elevator");
    }

    public Command armOutLED() {
        return led.armToPreset().andThen(armRotate.moveOut());
    }

    public Command moveArm(ArmRotatePresets preset) {
        return led.armToPreset().andThen(armRotate.moveTo(preset));
    }
    // Sets the rumble amount on controllers

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            Logger.recordOutput("Controllers/" + controller.getHID().getPort() + "/rumble", rumbleAmount);
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    // public Command intakeAuto() {
    // return moveElevator(ElevatorPresets.STOW,
    // ArmRotatePresets.INTAKE).andThen(coralManip.betterintake());
    // }

    // Resets all commands

    public Command resetCopilot() {
        return funnel.resetCmd().andThen(deepClimb.resetCmd());
    }
}