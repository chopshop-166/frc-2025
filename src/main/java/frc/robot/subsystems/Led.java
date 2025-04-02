package frc.robot.subsystems;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.patterns.AlliancePattern;
import com.chopshop166.chopshoplib.leds.patterns.FirePattern;
import com.chopshop166.chopshoplib.leds.patterns.FlashPattern;
import com.chopshop166.chopshoplib.leds.patterns.RainbowRoad;
import com.chopshop166.chopshoplib.leds.patterns.SpinPattern;
import com.chopshop166.chopshoplib.maps.LedMapBase;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.patterns.ElevatorFill;

public class Led extends LEDSubsystem {

    Color elevatorColor = new Color(128, 0, 128); // Make it purple!
    Color visionColorAligned = new Color(125, 218, 88); // Make it green!
    Color armColor = new Color(93, 226, 231); // Make it cyan!

    public Led(LedMapBase map) {
        super(map);
        // This one is length / 2 because the buffer has a mirrored other half
    }

    public Command colorAlliance() {
        return setGlobalPattern(new AlliancePattern());
    }

    public Command resetColor() {
        return setGlobalPattern(new Color(201, 198, 204));
    }

    public Command intakingStingray() {
        return setPattern("Intake", new FlashPattern(Color.kWhite, 0.15), "Flashing");
    }

    public Command elevatorToPreset() {
        return setPattern("Elevator", new SpinPattern(elevatorColor), "Moving");
    }

    public Command visionAligning() {
        return setPattern("Vision", new RainbowRoad(), "Aligning");
    }

    public Command visionAligned() {
        return setPattern("Vision", new FlashPattern(visionColorAligned, 0.125), "Aligned");
    }

    public Command deepClimbed() {
        return setPattern("Climber", new RainbowRoad(), "Climbed");
    }

    public Command armToPreset() {
        return setPattern("Arm", new SpinPattern(armColor), "Moving");
    }

    public Command armAtPreset() {
        return setPattern("Arm", new FlashPattern(armColor, 0.15), "At Preset");
    }

    // public Command elevatorAtPreset() {
    // return setPattern("Elevator", new FlashPattern(elevatorColor, 0.125), "At
    // preset");
    // }

    // public Command fillElevator() {
    // return setPattern("Elevator", new ElevatorFill(elevatorColor), "Fill");
    // }

    // public Command gamePieceAcquired() {
    // return setPattern("Intake", new FlashPattern(Color.kGreen, 0.125),
    // "Flashing");
    // }

    // public Command fire() {
    // return setPattern("Fun", new FirePattern(0), "Fire");
    // }

    // public Command awesomeColour() {
    // return setGlobalPattern(new Color(255, 32, 82));
    // }
}