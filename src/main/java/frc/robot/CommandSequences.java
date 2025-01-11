package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeDestage;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Outtake;

public class CommandSequences {

    Drive drive;
    Led led;
    AlgaeDestage algaeDestage;
    Outtake outtake;

    public CommandSequences(Drive drive, Led led, AlgaeDestage algaeDestage, Outtake outtake) {
        this.drive = drive;
        this.led = led;
        this.algaeDestage = algaeDestage;
        this.outtake = outtake;
    }

    public Command intake() {
        return outtake.spinIn();
    }

    public Command scoreCoral() {
        return outtake.spinOut();
    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }
}
