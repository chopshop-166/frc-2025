package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.leds.ColorFormat;
import com.chopshop166.chopshoplib.leds.SegmentConfig;
import com.chopshop166.chopshoplib.maps.CameraSource;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.VisionMap;
import com.chopshop166.chopshoplib.maps.WPILedMap;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorControllerGroup;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.ElevatorMap;
import frc.robot.maps.subsystems.FunnelMap;

@RobotMapFor("00:80:2F:40:A6:13")
public class Stingray extends RobotMap {
    @Override
    public SwerveDriveMap getDriveMap() {

        final double FLOFFSET = 45;
        final double FROFFSET = 18;
        final double RLOFFSET = 188;
        final double RROFFSET = 19;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(11.379);
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.smartCurrentLimit(30);
        steerConfig.idleMode(IdleMode.kCoast);
        steerConfig.inverted(true);
        frontLeftSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        frontRightSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        rearLeftSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        rearRightSteer.getMotorController().configure(steerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002),
                new PIDValues(0.05, 0.0, 0.0, 0.21));

        // All Distances are in Meters
        // Front Left Module
        final AnalogEncoder encoderFL = new AnalogEncoder(2, 360, FLOFFSET);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL::get, frontLeftSteer, new CSSparkFlex(3), MK4i_L2);

        // Front Right Module
        final AnalogEncoder encoderFR = new AnalogEncoder(3, 360, FROFFSET);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR::get, frontRightSteer, new CSSparkFlex(7), MK4i_L2);

        // Rear Left Module
        final AnalogEncoder encoderRL = new AnalogEncoder(0, 360, RLOFFSET);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL::get, rearLeftSteer, new CSSparkFlex(1), MK4i_L2);

        // Rear Right Module
        final AnalogEncoder encoderRR = new AnalogEncoder(1, 360, RROFFSET);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR::get, rearRightSteer, new CSSparkFlex(5), MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(15);

        final double maxRotationRadianPerSecond = 2 * Math.PI;

        RobotConfig config = new RobotConfig(50, 4.889, new ModuleConfig(
                0.0508, 6000, 1.0, DCMotor.getNeoVortex(1), 50, 1),
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY));
        PPHolonomicDriveController holonomicDrive = new PPHolonomicDriveController(new PIDConstants(2.0, 0.0, 0.05),
                new PIDConstants(1.0, 0.0, 0.0));

        return new SwerveDriveMap(frontLeft, frontRight, rearLeft, rearRight,
                maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, pigeonGyro2, config, holonomicDrive);
    }

    @Override
    public WPILedMap getLedMap() {
        var result = new WPILedMap(24, 0);
        var leds = result.ledBuffer;

        SegmentConfig Center = leds.segment(24, ColorFormat.GRB).tags("Intake", "Elevator", "Vision", "Fun");
        return result;
    }

    @Override
    public VisionMap getVisionMap() {

        return new VisionMap(
                new CameraSource("FL_Stingray_Cam",
                        new Transform3d(Units.inchesToMeters(9.43), Units.inchesToMeters(10.72),
                                Units.inchesToMeters(8.24),
                                new Rotation3d(0, Units.degreesToRadians(-68), Units.degreesToRadians(-16.76)))),
                new CameraSource("FR_Stingray_Cam",
                        new Transform3d(Units.inchesToMeters(
                                9.43),
                                Units.inchesToMeters(
                                        -10.72),
                                Units.inchesToMeters(8.24),
                                new Rotation3d(0, Units.degreesToRadians(-68), Units.degreesToRadians(16.76)))));
    }

    @Override
    public FunnelMap getFunnelMap() {
        CSSparkMax motor = new CSSparkMax(15);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return (new FunnelMap(motor, motor.getEncoder(), 0.5));
    }

    @Override
    public ElevatorMap getElevatorMap() {
        CSSparkFlex leftMotor = new CSSparkFlex(11);
        CSSparkFlex rightMotor = new CSSparkFlex(12);
        SparkFlexConfig configRight = new SparkFlexConfig();
        SparkFlexConfig configLeft = new SparkFlexConfig();

        configRight.follow(leftMotor.getMotorController());
        configRight.voltageCompensation(11.5);
        configRight.smartCurrentLimit(80);
        configRight.idleMode(IdleMode.kBrake);
        rightMotor.getMotorController().configure(configRight, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        configLeft.voltageCompensation(11.5);
        configLeft.smartCurrentLimit(80);
        configLeft.idleMode(IdleMode.kBrake);
        configLeft.encoder.velocityConversionFactor((((1 / 9.99) * Math.PI * 1.75) / 60) * 2)
                .positionConversionFactor(((1 / 9.99) * Math.PI * 1.75) * 2)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10);
        // Gear reduction is 22.2 sprocket diameter is 1.75 inches
        leftMotor.getMotorController().configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        ProfiledPIDController pid = new ProfiledPIDController(0.032, 0, 0,
                new Constraints(100, 250));
        pid.setTolerance(0.25);
        ElevatorFeedforward feedForward = new ElevatorFeedforward(0.001, 0.024, 0.0072);

        var elevatorMotors = new SmartMotorControllerGroup(leftMotor, rightMotor);

        ElevatorMap.PresetValues presets = preset -> switch (preset) {
            case STOW -> 1;
            case INTAKE -> 1;
            case SCOREL1 -> 13;
            case SCOREL1_TAKETWO -> 16;
            case SCOREL2 -> 21;
            case ALGAEL2 -> 24;
            case SCOREL3 -> 38;
            case ALGAEL3 -> 40;
            case SCOREL4, HIGHESTPOINT -> 59;
            default -> Double.NaN;
        };

        elevatorMotors.validateEncoderRate(.2, 10);
        return new ElevatorMap(elevatorMotors, leftMotor.getEncoder(), presets,
                new ValueRange(0, 59), new ValueRange(5, 55), pid, feedForward);
    }

    @Override
    public ArmRotateMap getArmRotateMap() {
        CSSparkFlex motor = new CSSparkFlex(10);
        DutyCycleEncoder absEncoder = new DutyCycleEncoder(0, 360, 0);
        SparkFlexConfig config = new SparkFlexConfig();
        absEncoder.setInverted(true);
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.encoder.quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10)
                .velocityConversionFactor((360.0 / 75) / 60)
                .positionConversionFactor(360.0 / 75);
        config.voltageCompensation(11.5);
        motor.getMotorController().configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        ProfiledPIDController pid = new ProfiledPIDController(0.01, 0, 0, new Constraints(90, 650));
        pid.setTolerance(3);
        ArmFeedforward feedForward = new ArmFeedforward(0.02, 0.0, 0.0011);

        ArmRotateMap.PresetValue presets = p -> switch (p) {
            case INTAKE -> 181;
            case SCOREL1, SCOREL2, SCOREL3, OUT -> 161;
            case SCOREL1_TAKETWO -> 121;
            case SCOREL4 -> 155;
            case STOW -> 181;
            case ALGAE -> 160;
            default -> Double.NaN;
        };

        return new ArmRotateMap(motor, absEncoder, presets, pid,
                new ValueRange(82, 181), new ValueRange(89, 181), feedForward);
    }

    @Override
    public CoralManipMap getCoralManipMap() {
        CSSparkMax motor = new CSSparkMax(9);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        CSDigitalInput sensor = new CSDigitalInput(1);
        sensor.setInverted(true);
        return new CoralManipMap(motor, sensor::get);
    }

    @Override
    public DeepClimbMap getDeepClimbMap() {
        CSSparkMax leftMotor = new CSSparkMax(14);
        CSSparkMax rightMotor = new CSSparkMax(13);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        leftMotor.getMotorController().configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        config.follow(leftMotor.getMotorController(), true);
        rightMotor.getMotorController().configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        return new DeepClimbMap(new SmartMotorControllerGroup(leftMotor, rightMotor),
                () -> false);
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB
        // stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
    }

}
