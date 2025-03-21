package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.maps.VisionMap.Data;

import frc.robot.maps.subsystems.MidocandriaMap;

public class Midocandria extends LoggedSubsystem<Data, MidocandriaMap> {
    public Midocandria(MidocandriaMap midocandriaMap) {
        super(new Data(), midocandriaMap);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void safeState() {

    }
}
