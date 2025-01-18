package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.CoralManipMap.Data;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralManip extends LoggedSubsystem<Data, CoralManipMap> {

    public CoralManip(CoralManipMap coralManipMap) {
        super(new Data(), coralManipMap);
    }

    @Override
    public void safeState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'safeState'");
    }

}
