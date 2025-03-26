package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.leds.ColorFormat;
import com.chopshop166.chopshoplib.leds.SegmentConfig;
import com.chopshop166.chopshoplib.maps.CameraSource;
import com.chopshop166.chopshoplib.maps.LedMapBase;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.VisionMap;
import com.chopshop166.chopshoplib.maps.WPILedMap;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

@RobotMapFor("00:80:2F:19:7B:A3")
public class Shrimp extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap() {

        // Remember to divide by 360
        // CAN ID
        final double FLOFFSET = 271.12; // 181.05;
        // CAN ID
        final double FROFFSET = 5.5; // 77.12 - 71.62;
        // CAN ID
        final double RLOFFSET = 321.77; // 185.41 + 136.36;
        // CAN ID
        final double RROFFSET = 256.88; // 321.43 + 115.45 - 180;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(6);
        final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(5));

        final CSSparkMax frontLeftSteer = new CSSparkMax(1);
        final CSSparkMax frontRightSteer = new CSSparkMax(3);
        final CSSparkMax rearLeftSteer = new CSSparkMax(5);
        final CSSparkMax rearRightSteer = new CSSparkMax(7);

        frontLeftSteer.setInverted(false);
        frontRightSteer.setInverted(false);
        rearLeftSteer.setInverted(false);
        rearRightSteer.setInverted(false);

        pigeonGyro.setInverted(true);

        // Configuration for MK4 with L2 speeds
        Configuration MK4_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.004, 0.00,
                        0.0002));

        // All Distances are in Meters
        // Front Left Module
        final AnalogEncoder encoderFL = new AnalogEncoder(0, 360, FLOFFSET);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                () -> encoderFL.get(), frontLeftSteer, new CSSparkMax(2), MK4_L2);

        // Front Right Module
        final AnalogEncoder encoderFR = new AnalogEncoder(3, 360, FROFFSET);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                () -> encoderFR.get(), frontRightSteer, new CSSparkMax(4), MK4_L2);

        // Rear Left Module
        final AnalogEncoder encoderRL = new AnalogEncoder(1, 360, RLOFFSET);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                () -> encoderRL.get(), rearLeftSteer, new CSSparkMax(6), MK4_L2);

        // Rear Right Module
        final AnalogEncoder encoderRR = new AnalogEncoder(2, 360, RROFFSET);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                () -> encoderRR.get(), rearRightSteer, new CSSparkMax(8), MK4_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(14);

        final double maxRotationRadianPerSecond = 2 * Math.PI;

        RobotConfig config = new RobotConfig(68, 5000, new ModuleConfig(
                0.1016, 6000, 1.0, DCMotor.getNEO(1), 50, 1),
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY));
        PPHolonomicDriveController holonomicDrive = new PPHolonomicDriveController(new PIDConstants(2.0, 0.0, 0.05),
                new PIDConstants(1.0, 0.0, 0.0));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro,
                config, holonomicDrive);
    }

    @Override
    public WPILedMap getLedMap() {
        var result = new WPILedMap(11, 0);
        var leds = result.ledBuffer;

        SegmentConfig spoiler = leds.segment(11, ColorFormat.GRB).tags("Intake", "Elevator", "Vision", "Fun");
        return result;
    }

    @Override
    public VisionMap getVisionMap() {
        // Front left and front right camera locations
        // Cam mounted 9.029 in. sideways of center(left), 9.029 in. forward of center,
        // 9.75 in. up from center. Mounted 0 degrees around x axis (roll), angled up
        // 65.752 degrees, and rotated side-to-side 45 degrees facing left
        Transform3d robotToCamFL = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.029), Units.inchesToMeters(9.029),
                        Units.inchesToMeters(9.75)),
                new Rotation3d(0, Units.degreesToRadians(-65.752), Units.degreesToRadians(45)));

        // Cam mounted 9.029 in. sideways of center(right), 9.029 in. forward of center,
        // 9.75 in. up from center. Mounted 0 degrees around x axis (roll), angled up
        // 64.752 degrees, and rotated side-to-side 45 degrees facing right
        Transform3d robotToCamFR = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.029), Units.inchesToMeters(9.029),
                        Units.inchesToMeters(9.75)),
                new Rotation3d(0, Units.degreesToRadians(-64.752), Units.degreesToRadians(-45)));

        return new VisionMap(new CameraSource("FLCamera", robotToCamFL),
                new CameraSource("FRCamera", robotToCamFR));
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging
    }
}
