package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.ElevatorMap;

@RobotMapFor("00:80:2F:40:A7:9D")
public class ColdStart extends RobotMap {
    @Override
    public SwerveDriveMap getDriveMap() {

        final double FLOFFSET = 46;
        final double FROFFSET = 270;
        final double RLOFFSET = 257.5;
        final double RROFFSET = 132.2 + 180;

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
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002));

        // All Distances are in Meters
        // Front Left Module
        final AnalogEncoder encoderFL = new AnalogEncoder(1, 360, FLOFFSET);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL::get, frontLeftSteer, new CSSparkFlex(3), MK4i_L2);

        // Front Right Module
        final AnalogEncoder encoderFR = new AnalogEncoder(3, 360, FROFFSET);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR::get, frontRightSteer, new CSSparkFlex(7), MK4i_L2);

        // Rear Left Module
        final AnalogEncoder encoderRL = new AnalogEncoder(2, 360, RLOFFSET);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL::get, rearLeftSteer, new CSSparkFlex(1), MK4i_L2);

        // Rear Right Module
        final AnalogEncoder encoderRR = new AnalogEncoder(0, 360, RROFFSET);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR::get, rearRightSteer, new CSSparkFlex(5), MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(15);

        final double maxRotationRadianPerSecond = Math.PI;

        RobotConfig config = new RobotConfig(68, 4.889, new ModuleConfig(
                0.1016, 6000, 1.0, DCMotor.getNeoVortex(1), 50, 1),
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
    public ElevatorMap getElevatorMap() {
        CSSparkFlex leftMotor = new CSSparkFlex(11);
        CSSparkFlex rightMotor = new CSSparkFlex(12);
        SparkFlexConfig configRight = new SparkFlexConfig();
        SparkFlexConfig configLeft = new SparkFlexConfig();

        configRight.follow(leftMotor.getMotorController());
        configRight.voltageCompensation(11.5);
        configRight.smartCurrentLimit(50);
        configRight.idleMode(IdleMode.kBrake);
        rightMotor.getMotorController().configure(configRight, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        configLeft.voltageCompensation(11.5);
        configLeft.smartCurrentLimit(50);
        configLeft.idleMode(IdleMode.kBrake);
        configLeft.encoder.velocityConversionFactor((((1 / 22.2) * Math.PI * 1.75) / 60) * 2);
        // Gear reduction is 22.2 sprocket diameter is 1.75 inches
        configLeft.encoder.positionConversionFactor(((1 / 22.2) * Math.PI * 1.75) * 2);
        leftMotor.getMotorController().configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leftMotor.validateEncoderRate(.2, 10);

        // we want to add this back
        // CSEncoder encoder = new CSEncoder(2, 3, false);

        ProfiledPIDController pid = new ProfiledPIDController(0.28, 0, 0,
                new Constraints(45, 150));
        pid.setTolerance(0.25);
        ElevatorFeedforward feedForward = new ElevatorFeedforward(0, 0.01, 0);

        return new ElevatorMap(leftMotor, leftMotor.getEncoder(),
                new ElevatorMap.ElevatorPresetValues(16, 5, 14, 29.5, 56, 57.5, 1),
                new ValueRange(0, 57.5), new ValueRange(3, 53), pid, feedForward);
    }

    @Override
    public CoralManipMap getCoralManipMap() {
        CSSparkMax leftWheels = new CSSparkMax(9);
        CSSparkMax rightWheels = new CSSparkMax(10);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        leftWheels.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        // Right is identical to left, but inverted
        config.inverted(true);
        rightWheels.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        CSDigitalInput sensor = new CSDigitalInput(9);
        return new CoralManipMap(leftWheels, rightWheels, sensor::get);
    }

    @Override
    public DeepClimbMap getDeepClimbMap() {
        CSSparkMax motor = new CSSparkMax(13);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CSDigitalInput sensor = new CSDigitalInput(8);
        return new DeepClimbMap(motor, sensor::get);

    }

    // @Override
    // public LedMapBase getLedMap() {
    // var result = new WPILedMap(1, 1);
    // var leds = result.ledBuffer;

    // SegmentConfig underglow = leds.segment(1).tags();
    // return result;
    // }

    @Override
    public void setupLogging() {
        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB
        // stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
