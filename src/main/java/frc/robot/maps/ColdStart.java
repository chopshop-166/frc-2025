package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.CSEncoder;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.maps.subsystems.CoralManipMap;
import frc.robot.maps.subsystems.DeepClimbMap;
import frc.robot.maps.subsystems.ElevatorMap;

@RobotMapFor("ColdStart")
public class ColdStart extends RobotMap {
    @Override
    public SwerveDriveMap getDriveMap() {

        final double FLOFFSET = 0;
        final double FROFFSET = 0;
        final double RLOFFSET = 0;
        final double RROFFSET = 0;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(10.875);
        final PigeonGyro2 pigeonGyro2 = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        frontLeftSteer.setInverted(true);
        frontRightSteer.setInverted(true);
        rearLeftSteer.setInverted(true);
        rearRightSteer.setInverted(true);

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.smartCurrentLimit(30);
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
        final CANcoder encoderFL = new CANcoder(2);
        CANcoderConfiguration encoderFLConfig = new CANcoderConfiguration();
        encoderFLConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderFLConfig.MagnetSensor.MagnetOffset = FLOFFSET;
        encoderFL.getConfigurator().apply(encoderFLConfig);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderFL), frontLeftSteer, new CSSparkMax(3), MK4i_L2);

        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderFRConfig.MagnetSensor.MagnetOffset = FROFFSET;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, new CSSparkMax(7), MK4i_L2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(3);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderRLConfig.MagnetSensor.MagnetOffset = RLOFFSET;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, new CSSparkMax(1), MK4i_L2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(1);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderRRConfig.MagnetSensor.MagnetOffset = RROFFSET;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, new CSSparkMax(5), MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(15);

        final double maxRotationRadianPerSecond = Math.PI;

        RobotConfig config = new RobotConfig(68, 5000, new ModuleConfig(
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
        configRight.smartCurrentLimit(30);
        rightMotor.getMotorController().configure(configRight, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        configLeft.voltageCompensation(11.5);
        configLeft.smartCurrentLimit(30);
        configLeft.encoder.velocityConversionFactor(((1 / 22.2) * Math.PI * 1.75) / 60);
        // Gear reduction is 22.2 sprocket diameter is 1.75 inches
        configLeft.encoder.positionConversionFactor((1 / 22.2) * Math.PI * 1.75);
        leftMotor.getMotorController().configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // we want to add this back
        // CSEncoder encoder = new CSEncoder(2, 3, false);

        ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
        pid.setTolerance(0.25);
        ElevatorFeedforward feedForward = new ElevatorFeedforward(0, 0, 0);

        return new ElevatorMap(leftMotor, leftMotor.getEncoder(),
                new ElevatorMap.ElevatorPresetValues(19.5, 5, 18, 38, 0),
                new ValueRange(0, 56), new ValueRange(3, 53), pid, feedForward);
    }

    @Override
    public CoralManipMap getCoralManipMap() {
        CSSparkMax leftWheels = new CSSparkMax(9);
        CSSparkMax rightWheels = new CSSparkMax(10);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        leftWheels.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
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
        motor.getMotorController().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CSDigitalInput sensor = new CSDigitalInput(8);
        return new DeepClimbMap(motor, sensor::get);

    }

}
