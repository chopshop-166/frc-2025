package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import frc.robot.maps.subsystems.MitocandriaMap;
import frc.robot.maps.subsystems.MitocandriaMap.Data;

public class Mitocandria extends LoggedSubsystem<Data, MitocandriaMap> {
    // This class is here to trigger the updateData function on the MitocandriaMap
    public Mitocandria(MitocandriaMap midocandriaMap) {
        super(new Data(), midocandriaMap);
    }

    @Override
    public void safeState() {

    }
}
