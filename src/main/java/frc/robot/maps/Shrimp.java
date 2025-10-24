package frc.robot.maps;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.leds.ColorFormat;
import com.chopshop166.chopshoplib.leds.SegmentConfig;
import com.chopshop166.chopshoplib.maps.CameraSource;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.VisionMap;
import com.chopshop166.chopshoplib.maps.WPILedMap;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.maps.subsystems.SwerveDriveMap;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

@RobotMapFor("00:80:2F:19:7B:A3")
public class Shrimp extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap(Subsystem driveSubsystem) {

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final Distance MODULE_OFFSET_XY = Inches.of(6);

        final PigeonGyro pigeonGyro = new PigeonGyro(new PigeonIMU(5));
        pigeonGyro.setInverted(true);

        SmartMotorControllerConfig steerConfig = new SmartMotorControllerConfig(driveSubsystem)
                .withMotorInverted(false)
                .withIdleMode(MotorMode.BRAKE)
                .withClosedLoopController(0.004, 0.0, 0.0002);

        SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(driveSubsystem)
                .withMotorInverted(false)
                .withIdleMode(MotorMode.BRAKE)
                .withWheelDiameter(Inches.of(3.95))
                // Configuration for MK4 with L2 speeds
                .withGearing(new MechanismGearing(GearBox.fromStages("14:50.0", "27:17", "15:45")));

        // Front Left
        final SmartMotorController frontLeftSteer = SmartMotorController.create(
                new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SmartMotorController frontLeftDrive = SmartMotorController.create(
                new SparkMax(2, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig(frontLeftDrive, frontLeftSteer)
                .withAbsoluteEncoder(new AnalogEncoder(0, 360, 271.12)::get)
                .withLocation(MODULE_OFFSET_XY, MODULE_OFFSET_XY.unaryMinus());

        // Front right
        final SmartMotorController frontRightSteer = SmartMotorController.create(
                new SparkMax(3, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SmartMotorController frontRightDrive = SmartMotorController.create(
                new SparkMax(4, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SwerveModuleConfig frontRightConfig = new SwerveModuleConfig(frontRightDrive, frontRightSteer)
                .withAbsoluteEncoder(new AnalogEncoder(3, 360, 5.5)::get)
                .withLocation(MODULE_OFFSET_XY, MODULE_OFFSET_XY);

        // Back left
        final SmartMotorController backLeftSteer = SmartMotorController.create(
                new SparkMax(5, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SmartMotorController backLeftDrive = SmartMotorController.create(
                new SparkMax(6, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SwerveModuleConfig backLeftConfig = new SwerveModuleConfig(backLeftDrive, backLeftSteer)
                .withAbsoluteEncoder(new AnalogEncoder(1, 360, 321.77)::get)
                .withLocation(MODULE_OFFSET_XY.unaryMinus(), MODULE_OFFSET_XY);

        // Back right
        final SmartMotorController backRightSteer = SmartMotorController.create(
                new SparkMax(7, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SmartMotorController backRightDrive = SmartMotorController.create(
                new SparkMax(8, MotorType.kBrushless), DCMotor.getNEO(1), steerConfig);
        final SwerveModuleConfig backRightConfig = new SwerveModuleConfig(backRightDrive, backRightSteer)
                .withAbsoluteEncoder(new AnalogEncoder(2, 360, 256.88)::get)
                .withLocation(MODULE_OFFSET_XY, MODULE_OFFSET_XY);

        final SwerveDriveConfig swerveDriveConfig = new SwerveDriveConfig(driveSubsystem,
                new SwerveModule(frontLeftConfig),
                new SwerveModule(frontRightConfig),
                new SwerveModule(backLeftConfig),
                new SwerveModule(backRightConfig))
                .withGyro(() -> pigeonGyro.getRotation2d().getMeasure())
                .withMaximumChassisSpeed(FeetPerSecond.of(3), DegreesPerSecond.of(360))
                .withTelemetry(TelemetryVerbosity.HIGH);

        RobotConfig config = new RobotConfig(68, 5000, new ModuleConfig(
                0.1016, 6000, 1.0, DCMotor.getNEO(1), 50, 1),
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY.unaryMinus()),
                new Translation2d(MODULE_OFFSET_XY.unaryMinus(), MODULE_OFFSET_XY),
                new Translation2d(MODULE_OFFSET_XY.unaryMinus(), MODULE_OFFSET_XY.unaryMinus()));
        PPHolonomicDriveController holonomicDrive = new PPHolonomicDriveController(
                new PIDConstants(2.0, 0.0, 0.05),
                new PIDConstants(1.0, 0.0, 0.0));

        return new SwerveDriveMap(swerveDriveConfig, config, holonomicDrive);
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
