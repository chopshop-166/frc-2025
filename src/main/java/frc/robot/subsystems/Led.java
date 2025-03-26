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

    Color elevatorColor = new Color(112, 255, 248);
    Color visionColor = new Color(255, 32, 82);

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

    public Command intaking() {
        return setPattern("Intake", new SpinPattern(), "Spinning");
    }

    public Command intakingStingray() {
        return setPattern("Intake", new FlashPattern(Color.kWhite, 0.15), "Flashing");
    }

    public Command gamePieceAcquired() {
        return setPattern("Intake", new FlashPattern(Color.kGreen, 0.125), "Flashing");
    }

    public Command elevatorToPreset() {
        return setPattern("Elevator", new SpinPattern(elevatorColor), "Moving");
    }

    public Command elevatorAtPreset() {
        return setPattern("Elevator", new FlashPattern(elevatorColor, 0.125), "At preset");
    }

    public Command fillElevator() {
        return setPattern("Elevator", new ElevatorFill(elevatorColor), "Fill");
    }

    public Command visionAligning() {
        return setPattern("Vision", new SpinPattern(visionColor), "Aligning");
    }

    public Command visionAligned() {
        return setPattern("Vision", new FlashPattern(visionColor, 0.125), "Aligned");
    }

    public Command starPower() {
        return setPattern("Fun", new RainbowRoad(), "Rainbow");
    }

    public Command fire() {
        return setPattern("Fun", new FirePattern(0), "Fire");
    }

    public Command awesomeColour() {
        return setGlobalPattern(new Color(255, 32, 82));
    }
}