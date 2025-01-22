package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.ArmRotateMap.Data;

public class ArmRotate extends LoggedSubsystem<Data, ArmRotateMap> {

    public ArmRotate(ArmRotateMap armRotateMap) {
        super(new Data(), armRotateMap);
    }

    @Override
    public void safeState() {

    }
}
