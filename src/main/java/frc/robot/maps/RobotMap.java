package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.LedMapBase;
import com.chopshop166.chopshoplib.maps.MockLedMap;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.VisionMap;

import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.MitocandriaMap;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.UndertakerMap;

public class RobotMap {

    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public VisionMap getVisionMap() {
        return new VisionMap();
    }

    public LedMapBase getLedMap() {
        return new MockLedMap();
    }

    public ArmRotateMap getArmRotateMap() {
        return new ArmRotateMap();
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();
    }

    public UndertakerMap getUndertakerMap() {
        return new UndertakerMap();
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public MitocandriaMap getMitocandriaMap() {
        return new MitocandriaMap();
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
