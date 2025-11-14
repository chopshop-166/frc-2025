package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Undertaker;
import frc.robot.subsystems.Shooter.Speeds;

public class CommandSequences {

    Drive drive;
    Intake intake;
    Shooter shooter;
    ArmRotate armRotate;
    Undertaker undertaker;

    public CommandSequences(Drive drive, Intake intake, Shooter shooter, ArmRotate armRotate,
            Undertaker undertaker) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.armRotate = armRotate;
        this.undertaker = undertaker;
    }

    // make sequences for intake and shooter.

    public Command intake() {
        return intake.intakeGamePiece();
    }

    public Command shooterSpeed(Speeds speed) {
        return shooter.setSpeed(speed);
    }

    public Command armRotatePreset(ArmRotatePresets presets) {
        return armRotate.moveTo(presets);
    }

    public Command moveAndIntake() {
        return this.armRotatePreset(ArmRotatePresets.INTAKE)
                .andThen(this.intake().deadlineFor(undertaker.spinIn()));
    }

    public Command feedShoot() {
        return shooterSpeed(Speeds.SUBWOOFER_SHOT).andThen(waitSeconds(.2),
                intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF));
    }

    public Command outtake() {
        return intake.spinOut().alongWith(
                undertaker.spinOut());

    }

    public Command moveToIntake() {
        return armRotate.moveTo(ArmRotatePresets.INTAKE);
    }

    public Command moveToSpeaker() {
        return armRotate.moveTo(ArmRotatePresets.SHOOT_HIGH);
    }

    public Command rotateToIntake() {
        return this.armRotatePreset(ArmRotatePresets.INTAKE);
    }

    public Command charge(Speeds speed, ArmRotatePresets angle) {
        return this.shooterSpeed(speed).alongWith(armRotatePreset(angle));
    }

    public Command release() {
        return this.intake.feedShooter().andThen(this.shooterSpeed(Speeds.OFF),
                armRotatePreset(ArmRotatePresets.INTAKE));
    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }
}
