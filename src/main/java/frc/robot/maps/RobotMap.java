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
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.FunnelMap;
import frc.robot.maps.subsystems.MitocandriaMap;

public class RobotMap {

    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public FunnelMap getFunnelMap() {
        return new FunnelMap();
    }

    public VisionMap getVisionMap() {
        return new VisionMap();
    }

    public LedMapBase getLedMap() {
        return new MockLedMap();
    }

    public CoralManipMap getCoralManipMap() {
        return new CoralManipMap();
    }

    public ElevatorMap getElevatorMap() {
        return new ElevatorMap();
    }

    public DeepClimbMap getDeepClimbMap() {
        return new DeepClimbMap();
    }

    public ArmRotateMap getArmRotateMap() {
        return new ArmRotateMap();
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
