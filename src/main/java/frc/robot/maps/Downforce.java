package frc.robot.maps;

import java.io.ObjectInputFilter.Config;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.ValueRange;
import com.chopshop166.chopshoplib.digital.CSDigitalInput;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule;
import com.chopshop166.chopshoplib.drive.SDSSwerveModule.Configuration;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.motors.CSSpark;
import com.chopshop166.chopshoplib.motors.CSSparkFlex;
import com.chopshop166.chopshoplib.motors.CSSparkMax;
import com.chopshop166.chopshoplib.motors.SmartMotorControllerGroup;
import com.chopshop166.chopshoplib.sensors.CSEncoder;
import com.chopshop166.chopshoplib.sensors.CSFusedEncoder;
import com.chopshop166.chopshoplib.sensors.CtreEncoder;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro;
import com.chopshop166.chopshoplib.sensors.gyro.PigeonGyro2;
import com.chopshop166.chopshoplib.states.PIDValues;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.UndertakerMap;

@RobotMapFor("Downforce")
public class Downforce extends RobotMap {

    @Override
    public SwerveDriveMap getDriveMap() {

        // Use Phoenix tuner
        // CAN ID 2
        final double FLOFFSET = -0.48;
        // CAN ID 4
        final double FROFFSET = -0.26;
        // CAN ID 1
        final double RLOFFSET = -0.36;
        // CAN ID 3
        final double RROFFSET = -0.959;

        // Value taken from CAD as offset from center of module base pulley to center
        // of the robot
        final double MODULE_OFFSET_XY = Units.inchesToMeters(10.875);
        final PigeonGyro2 pigeonGyro = new PigeonGyro2(1);

        final CSSparkMax frontLeftSteer = new CSSparkMax(4);
        final CSSparkMax frontRightSteer = new CSSparkMax(8);
        final CSSparkMax rearLeftSteer = new CSSparkMax(2);
        final CSSparkMax rearRightSteer = new CSSparkMax(6);

        CSSparkFlex frontLeftDrive = new CSSparkFlex(3);
        CSSparkFlex frontRightDrive = new CSSparkFlex(7);
        CSSparkFlex rearLeftDrive = new CSSparkFlex(1);
        CSSparkFlex rearRightDrive = new CSSparkFlex(5);

        frontLeftSteer.setInverted(true);
        frontRightSteer.setInverted(true);
        rearLeftSteer.setInverted(true);
        rearRightSteer.setInverted(true);

        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.MountPoseRoll = 180;
        pigeonGyro.getRaw().getConfigurator().apply(pigeonConfig, MODULE_OFFSET_XY);

        // Configuration for MK4i with L2 speeds
        Configuration MK4i_L2 = new Configuration(SDSSwerveModule.MK4_V2.gearRatio,
                SDSSwerveModule.MK4_V2.wheelDiameter, new PIDValues(0.011, 0.00, 0.0002),
                new PIDValues(0.05, 0.0, 0.0, 0.21));

        // All Distances are in Meters
        // Front left module
        final CANcoder encoderFL = new CANcoder(2);
        CANcoderConfiguration encoderFLConfig = new CANcoderConfiguration();
        encoderFLConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderFLConfig.MagnetSensor.MagnetOffset = FLOFFSET;
        encoderFL.getConfigurator().apply(encoderFLConfig);
        final SDSSwerveModule frontLeft = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderFL), frontLeftSteer, frontLeftDrive, MK4i_L2);
        // Front Right Module
        final CANcoder encoderFR = new CANcoder(4);
        CANcoderConfiguration encoderFRConfig = new CANcoderConfiguration();
        encoderFRConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderFRConfig.MagnetSensor.MagnetOffset = FROFFSET;
        encoderFR.getConfigurator().apply(encoderFRConfig);
        final SDSSwerveModule frontRight = new SDSSwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderFR), frontRightSteer, frontRightDrive, MK4i_L2);

        // Rear Left Module
        final CANcoder encoderRL = new CANcoder(1);
        CANcoderConfiguration encoderRLConfig = new CANcoderConfiguration();
        encoderRLConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderRLConfig.MagnetSensor.MagnetOffset = RLOFFSET;
        encoderRL.getConfigurator().apply(encoderRLConfig);
        final SDSSwerveModule rearLeft = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                new CtreEncoder(encoderRL), rearLeftSteer, rearLeftDrive, MK4i_L2);

        // Rear Right Module
        final CANcoder encoderRR = new CANcoder(3);
        CANcoderConfiguration encoderRRConfig = new CANcoderConfiguration();
        encoderRRConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderRRConfig.MagnetSensor.MagnetOffset = RROFFSET;
        encoderRR.getConfigurator().apply(encoderRRConfig);
        final SDSSwerveModule rearRight = new SDSSwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                new CtreEncoder(encoderRR), rearRightSteer, rearRightDrive, MK4i_L2);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(3);

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
    public ArmRotateMap getArmRotateMap() {
        CSSparkMax leftMotor = new CSSparkMax(13);
        CSSparkMax rightMotor = new CSSparkMax(14);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        leftMotor.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        config.follow(leftMotor.getMotorController(), true);
        rightMotor.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        CSEncoder encoder = new CSEncoder(2, 3, false);
        encoder.setDistancePerPulse(360.0 / 2048.0);
        DutyCycleEncoder absEncoder = new DutyCycleEncoder(1, 360, 60.2);
        absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        IEncoder absIEncoder = new IEncoder() {
            @Override
            public double getAbsolutePosition() {
                return absEncoder.get();
            }

            @Override
            public double getDistance() {
                return 0;
            }

            @Override
            public double getRate() {
                return 0;
            }

            @Override
            public void reset() {
            }
        };
        // Adjust this to fix absolute encoder angle. If at your zero angle,
        // just put in that number, no need to make it negative
        ProfiledPIDController pid = new ProfiledPIDController(0.02, 0.0, 0.0, new Constraints(120, 500));
        pid.setTolerance(2);
        ArmFeedforward feedForward = new ArmFeedforward(0, 0.03, 0.35, 0);

        ArmRotateMap.PresetValue presets = p -> switch (p) {
            case INTAKE -> -13;
            case SHOOT_HIGH -> 5;
            case SHOOT_LOW -> 23;
            default -> Double.NaN;
        };

        return new ArmRotateMap(new SmartMotorControllerGroup(leftMotor, rightMotor),
                absIEncoder, presets, pid,
                // Hard limits
                new ValueRange(-13.75, 87),
                // Soft limits
                new ValueRange(0, 73),
                feedForward);
    }

    @Override
    public IntakeMap getIntakeMap() {
        CSSparkMax topRoller = new CSSparkMax(12);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(30);
        topRoller.getMotorController().configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        CSDigitalInput sensor = new CSDigitalInput(0);
        return new IntakeMap(topRoller, sensor::get, 0.3);
    }

    @Override
    public UndertakerMap getUndertakerMap() {
        CSSparkFlex topRoller = new CSSparkFlex(16);
        CSSparkFlex bottomRoller = new CSSparkFlex(15);
        SparkMaxConfig configTop = new SparkMaxConfig();
        SparkMaxConfig configBottom = new SparkMaxConfig();
        configTop.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(50);
        topRoller.getMotorController().configure(configTop, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        configBottom.follow(topRoller.getMotorController());
        return new UndertakerMap(new SmartMotorControllerGroup(topRoller,
                bottomRoller));
    }

    @Override
    public ShooterMap getShooterMap() {
        CSSparkFlex rightWheels = new CSSparkFlex(11);
        CSSparkFlex leftWheels = new CSSparkFlex(10);
        SparkMaxConfig configLeft = new SparkMaxConfig();
        SparkMaxConfig configRight = new SparkMaxConfig();

        configRight.smartCurrentLimit(50);
        configLeft.smartCurrentLimit(50);
        configLeft.inverted(true);
        configRight.closedLoop.p(0.003).i(0).d(0).velocityFF(0.000182);
        configLeft.closedLoop.p(0.0015).i(0).d(0).velocityFF(0.000174);
        configRight.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);
        configLeft.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);

        leftWheels.getMotorController().configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightWheels.getMotorController().configure(configRight, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightWheels.setControlType(ControlType.kVelocity);
        leftWheels.setControlType(ControlType.kVelocity);
        return new ShooterMap(rightWheels, leftWheels, true);
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging
    }

}
