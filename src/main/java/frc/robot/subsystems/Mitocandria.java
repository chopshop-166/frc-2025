package frc.robot.subsystems;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.maps.subsystems.MitocandriaMap;
import frc.robot.maps.subsystems.MitocandriaMap.Data;

public class Mitocandria extends LoggedSubsystem<Data, MitocandriaMap> {

    private Alert orangePiAlert = new Alert(
            "Orange Pi not drawing expected power, verify that Orange Pi is plugged in.", AlertType.kError);

    // This class is here to trigger the updateData function on the MitocandriaMap
    public Mitocandria(MitocandriaMap midocandriaMap) {
        super(new Data(), midocandriaMap);
    }

    @Override
    public void periodic() {
        orangePiAlert.set(getData().usb_port_1_current < 0.5);
    }

    @Override
    public void safeState() {

    }
}
