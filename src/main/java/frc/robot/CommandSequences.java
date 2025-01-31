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

    public Command intake() {
        return led.intaking().andThen(elevator.moveTo(ElevatorPresets.INTAKE), led.elevatorAtPreset(),
                coralManip.intake(), led.gamePieceAquired());

    }

    public Command moveElevator(ElevatorPresets level) {
        return led.elevatorToPreset().andThen(elevator.moveTo(level), led.elevatorAtPreset());
    }

    public Command score() {
        return coralManip.score().andThen(led.elevatorToPreset(), elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    public Command scoreCoral(ElevatorPresets level) {
        return led.elevatorToPreset().andThen(elevator.moveTo(level), led.elevatorAtPreset(), coralManip.score());
    }

    public Command scoreL1() {
        return coralManip.scoreL1().andThen(led.elevatorToPreset(), elevator.moveTo(ElevatorPresets.STOW),
                led.elevatorAtPreset());
    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    public Command resetAll() {
        return drive.resetCmd().andThen(coralManip.resetCmd(), algaeDestage.resetCmd());
    }
}
