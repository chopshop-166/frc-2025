package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.ElevatorMap.Data;

import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends LoggedSubsystem<Data, ElevatorMap> {

    public Elevator(ElevatorMap elevatorMap) {
        super(new Data(), elevatorMap);
    }

    @Override
    public void safeState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'safeState'");
    }

}
