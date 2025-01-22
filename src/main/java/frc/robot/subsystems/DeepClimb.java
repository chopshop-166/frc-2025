package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
 
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.DeepClimbMap.Data;
import edu.wpi.first.wpilibj2.command.Command;

public class DeepClimb extends LoggedSubsystem<Data, DeepClimbMap> {

    public DeepClimb(DeepClimbMap deepClimbMap) {
        super(new Data(), deepClimbMap);
    }

    @Override
    public void safeState() {
        // no safestate to set
    }

}
