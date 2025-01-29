package frc.robot.subsystems;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.Pattern;
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

    public Command gamePieceAquired() {
        return setPattern("Intake", new FlashPattern(Color.kGreen, 0.125), "Flashing");
    }

    public Command elevatorToPreset() {
        return setPattern("Elevator", new SpinPattern(new Color(112, 255, 248)), "Moving");
    }

    public Command elevatorAtPreset() {
        return setPattern("Elevator", new FlashPattern(new Color(112, 255, 248), 0.125), "At preset");
    }

    public Command fillElevator() {
        return setPattern("Elevator", new ElevatorFill(), "Fill");
    }

    public Command visionAligning() {
        return setPattern("Vision", new SpinPattern(new Color(255, 32, 82)), "Aligning");
    }

    public Command visionAligned() {
        return setPattern("Vision", new FlashPattern(new Color(255, 32, 82), 0.125), "Aligned");
    }

    public Command starPower() {
        return setPattern("Fun", new RainbowRoad(), "Rainbow");
    }

    public Command fire() {
        return setPattern("Fun", new FirePattern(0), "Fire");
    }

    public Command awesome() {
        return setGlobalPattern(new Color(255, 32, 82));
    }
}