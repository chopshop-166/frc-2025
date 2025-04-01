package frc.robot.maps.subsystems;

import java.util.Optional;

import com.chopshop166.chopshoplib.logging.DataWrapper;
import com.chopshop166.chopshoplib.logging.LoggableMap;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class MitocandriaMap implements LoggableMap<MitocandriaMap.Data> {

    private Optional<MitoCANdria> mitocandria;
    private Alert mitoAlert = new Alert("Failed to retrieve data from MitoCANdria", AlertType.kWarning);

    public MitocandriaMap() {
        mitocandria = Optional.empty();
    }

    public MitocandriaMap(MitoCANdria mito) {
        this.mitocandria = Optional.of(mito);
    }

    @Override
    public void updateData(Data data) {
        if (!mitocandria.isPresent()) {
            return;
        }
        MitoCANdria mito = mitocandria.get();
        try {
            mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB1)
                    .ifPresent(current -> data.usb_port_1_current = current);
            mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_USB1)
                    .ifPresent(voltage -> data.usb_port_1_voltage = voltage);
            mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_USB1)
                    .ifPresent(enabled -> data.usb_port_1_enabled = enabled);

            mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB2)
                    .ifPresent(current -> data.usb_port_2_current = current);
            mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_USB2)
                    .ifPresent(voltage -> data.usb_port_2_voltage = voltage);
            mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_USB2)
                    .ifPresent(enabled -> data.usb_port_2_enabled = enabled);

            mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                    .ifPresent(current -> data.aux_5v_port_1_current = current);
            mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                    .ifPresent(voltage -> data.aux_5v_port_1_voltage = voltage);
            mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                    .ifPresent(enabled -> data.aux_5v_port_1_enabled = enabled);

            mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                    .ifPresent(current -> data.aux_5v_port_2_current = current);
            mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                    .ifPresent(voltage -> data.aux_5v_port_2_voltage = voltage);
            mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                    .ifPresent(enabled -> data.aux_5v_port_2_enabled = enabled);

            mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ)
                    .ifPresent(current -> data.adjustable_port_current = current);
            mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ)
                    .ifPresent(voltage -> data.adjustable_port_voltage = voltage);
            mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ)
                    .ifPresent(enabled -> data.adjustable_port_enabled = enabled);
            mito.getChannelVoltageSetpoint(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ)
                    .ifPresent(setpoint -> data.adjustable_port_setpoint = setpoint);

            // Clear the alert if we succesfully got data
            mitoAlert.set(false);
        } catch (Exception e) {
            mitoAlert.set(true);
        }
    }

    public static class Data extends DataWrapper {
        public double usb_port_1_current = 0;
        public double usb_port_1_voltage = 0;
        public double usb_port_1_enabled = 0;
        public double usb_port_2_current = 0;
        public double usb_port_2_voltage = 0;
        public double usb_port_2_enabled = 0;
        public double aux_5v_port_1_current = 0;
        public double aux_5v_port_1_voltage = 0;
        public double aux_5v_port_1_enabled = 0;
        public double aux_5v_port_2_current = 0;
        public double aux_5v_port_2_voltage = 0;
        public double aux_5v_port_2_enabled = 0;
        public double adjustable_port_current = 0;
        public double adjustable_port_voltage = 0;
        public double adjustable_port_enabled = 0;
        public double adjustable_port_setpoint = 0;
    }

}
