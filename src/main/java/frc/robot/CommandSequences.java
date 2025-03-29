package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

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
        return Commands.either(Commands.none(), armRotate.moveOut().andThen(led.elevatorToPreset()),
                () -> elevator.atPreset(ElevatorPresets.INTAKE) || elevator.atPreset(ElevatorPresets.STOW))
                .andThen(elevator.clearPreset(),
                        elevator.moveTo(ElevatorPresets.INTAKE))
                .withName("Intake");
    }

    public Command intakeBottom() {
        return armRotate.moveToNonOwning(ArmRotatePresets.INTAKE)
                .alongWith(coralManip.betterintake())
                .withName("Intake Bottom");
    }

    // Moves elevator to set coral preset

    public Command moveElevator(ElevatorPresets level, ArmRotatePresets preset) {
        return led.elevatorToPreset().andThen(armRotate.moveOut(), elevator.moveTo(level),
                armRotate.moveTo(preset)).withName("Move Elevator");
    }

    // Sets the rumble amount on controllers

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    // Resets all commands

    public Command resetCopilot() {
        return funnel.resetCmd().andThen(deepClimb.resetCmd());
    }
}