package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;
import frc.robot.subsystems.AlgaeDestage;
import frc.robot.subsystems.CoralManip;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.ArmRotate;

public class CommandSequences {

    Drive drive;
    Led led;
    AlgaeDestage algaeDestage;
    CoralManip coralManip;
    Elevator elevator;
    ArmRotate armRotate;

    public CommandSequences(Drive drive, Led led, AlgaeDestage algaeDestage, CoralManip coralManip, Elevator elevator,
            ArmRotate armRotate) {
        this.drive = drive;
        this.led = led;
        this.algaeDestage = algaeDestage;
        this.coralManip = coralManip;
        this.elevator = elevator;
        this.armRotate = armRotate;
    }

    // Moves elevator to intake preset and intakes when preset has been reached
    // Intakes until sensor is tripped, LEDs indicate that game piece is acquired

    public Command intake() {
        return armRotate.moveIntaking().andThen(led.elevatorToPreset(),
                elevator.moveTo(ElevatorPresets.INTAKE), led.elevatorAtPreset(),
                armRotate.moveTo(ArmRotatePresets.INTAKE),
                led.intaking(), coralManip.intake(), led.gamePieceAcquired());
    }

    public Command intakeBottom() {
        return armRotate.moveTo(ArmRotatePresets.INTAKE).alongWith(coralManip.intake());
    }

    // Moves elevator to set coral preset

    public Command moveElevator(ElevatorPresets level, ArmRotatePresets preset) {
        return led.elevatorToPreset().andThen(armRotate.moveTo(ArmRotatePresets.OUT), elevator.moveTo(level),
                led.elevatorAtPreset(), armRotate.moveTo(preset));
    }

    // Scores on set coral preset, then stows elevator

    public Command score(ArmRotatePresets preset) {
        return coralManip.score().andThen((led.elevatorToPreset()),
                elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    // Moves elevator to set coral preset and scores coral automatically

    public Command scoreCoralAuto(ElevatorPresets level) {
        return led.elevatorToPreset().andThen(elevator.moveTo(level), led.elevatorAtPreset(), coralManip.score());
    }

    // Moves elevator to L1 preset
    // L1 is different from the other scoring presets which is why its has it own
    // move to and score command

    public Command positionAuto(ElevatorPresets level) {
        return elevator.moveTo(level);
    }

    public Command scoreL1Auto() {
        return scoreL1();
    }

    public Command positionL2Auto() {
        return moveElevator(ElevatorPresets.SCOREL2, ArmRotatePresets.SCOREL2);
    }

    public Command positionL3Auto() {
        return moveElevator(ElevatorPresets.SCOREL3, ArmRotatePresets.SCOREL3);
    }

    public Command positionL4Auto() {
        return moveElevator(ElevatorPresets.SCOREL4, ArmRotatePresets.SCOREL4);
    }

    // Scores on L1 preset, then stows elevator

    public Command scoreL1() {
        return armRotate.moveTo(ArmRotatePresets.SCOREL1).andThen(coralManip.scoreL1(), (led.elevatorToPreset()),
                elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    // Scores on L4 preset, moving elevator up to score and then stowing elevator

    public Command scoreL4() {
        return coralManip.scoreL4().andThen(led.elevatorToPreset(),
                // kick elevator up so that it actually scores (knocks coral onto reef)
                armRotate.moveTo(ArmRotatePresets.SCOREL4),
                elevator.moveTo(ElevatorPresets.HIGHESTPOINT),
                waitSeconds(0.5),
                elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    // Sets the rumble amount on controllers

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    // Resets all commands

    public Command resetAll() {
        return drive.resetCmd().andThen(coralManip.resetCmd(), algaeDestage.resetCmd());
    }
}