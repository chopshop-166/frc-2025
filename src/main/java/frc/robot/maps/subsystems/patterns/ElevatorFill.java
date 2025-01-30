package frc.robot.maps.subsystems.patterns;

import com.chopshop166.chopshoplib.leds.AnimatedPattern;
import com.chopshop166.chopshoplib.leds.SegmentBuffer;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.AlgaeDestageMap.Data;
import frc.robot.subsystems.Elevator;

public class ElevatorFill extends AnimatedPattern {
    private final Color color;
    /** The position of the indicator. */
    private int ledPosition;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    DoubleSubscriber heightSub = instance.getDoubleTopic("Elevator/Height").subscribe(0);

    /** Constructor. */
    public ElevatorFill() {
        this(new Color(112, 255, 248));
    }

    /**
     * Constructor.
     * 
     * @param color The color of the dot.
     */
    public ElevatorFill(final Color color) {
        super(0);
        this.color = color;
    }

    @Override
    public void initialize(final SegmentBuffer buffer) {
        super.initialize(buffer);
        this.ledPosition = 0;
    }

    @Override
    public void animate(final SegmentBuffer buffer) {

        buffer.setAll(Color.kBlack);
        this.ledPosition = (int) Math.floor(heightSub.getAsDouble() * buffer.getLength());

        for (int i = 0; i <= this.ledPosition; i++) {
            buffer.set(i, this.color);
        }

    }

    @Override
    public String toString() {
        return String.format("FillPattern(%s)", this.color.toString());
    }
}
