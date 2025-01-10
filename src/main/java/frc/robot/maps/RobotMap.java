package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.LedMapBase;
import com.chopshop166.chopshoplib.maps.MockLedMap;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;

import frc.robot.maps.subsystems.patterns.AlgaeDestageMap;
import frc.robot.maps.subsystems.patterns.OuttakeMap;

public class RobotMap {

    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public LedMapBase getLedMap() {
        return new MockLedMap();
    }

    public OuttakeMap getOuttakeMap() {
        return new OuttakeMap();
    }

    public AlgaeDestageMap getAlgaeDestageMap() {
        return new AlgaeDestageMap();
    }

    public void setupLogging() {
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
}
