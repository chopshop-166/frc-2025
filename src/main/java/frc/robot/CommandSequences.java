package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;
import frc.robot.subsystems.AlgaeDestage;
import frc.robot.subsystems.CoralManip;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Outtake;

public class CommandSequences {

    Drive drive;
    Led led;
    AlgaeDestage algaeDestage;
    CoralManip coralManip;
    Elevator elevator;

    public CommandSequences(Drive drive, Led led, AlgaeDestage algaeDestage, CoralManip coralManip, Elevator elevator) {
        this.drive = drive;
        this.led = led;
        this.algaeDestage = algaeDestage;
        this.coralManip = coralManip;
        this.elevator = elevator;
    }

    // Moves elevator to intake preset and intakes when preset has been reached
    // Intakes until sensor tripped, leds indicate that game piece is aquired

    public Command intake() {
        return led.elevatorToPreset().andThen(elevator.moveTo(ElevatorPresets.INTAKE), led.elevatorAtPreset(),
                led.intaking(), coralManip.intake(), led.gamePieceAquired());
    }

    // Moves elevtor to set coral preset

    public Command moveElevator(ElevatorPresets level) {
        return led.elevatorToPreset().andThen(elevator.moveTo(level), led.elevatorAtPreset());
    }

    // Scores on set coral preset, then stows elevator

    public Command score() {
        return coralManip.score().andThen(led.elevatorToPreset(), elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    // Moves elevator to set coral preset and scores coral automatically

    public Command scoreCoralAuto(ElevatorPresets level) {
        return led.elevatorToPreset().andThen(elevator.moveTo(level), led.elevatorAtPreset(), coralManip.score());
    }

    // Moves elevator to L1 preset
    // L1 is differnt from the other scoring presets which is why its has its own
    // move to and score command

    public Command scoreL1Auto() {
        return moveElevator(ElevatorPresets.SCOREL1).andThen(scoreL1());
    }

    // Scores on L1 preset, then stows elevator

    public Command scoreL1() {
        return coralManip.scoreL1().andThen(led.elevatorToPreset(), elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    // Sets the rumble amount on controlers

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    // Resets all comands

    public Command resetAll() {
        return drive.resetCmd().andThen(coralManip.resetCmd(), algaeDestage.resetCmd(), elevator.resetCmd());
    }
}