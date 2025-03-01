package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.logging.LoggedSubsystem;
import com.chopshop166.chopshoplib.logging.data.SwerveDriveData;
import com.chopshop166.chopshoplib.maps.CameraSource;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;
import com.chopshop166.chopshoplib.maps.VisionMap;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends LoggedSubsystem<SwerveDriveData, SwerveDriveMap> {

    public final SwerveDriveKinematics kinematics;
    private final VisionMap visionMap;

    private final double maxDriveSpeedMetersPerSecond;
    private final double maxRotationRadiansPerSecond;
    private final double SPEED_COEFFICIENT = 1;
    private final double ROTATION_COEFFICIENT = 1;
    private final double ROTATION_KP = 0.05;
    private final double ROTATION_KS = 0.19;
    final Modifier DEADBAND = Modifier.scalingDeadband(0.1);

    ProfiledPIDController rotationPID = new ProfiledPIDController(0.05, 0.0002, 0.000, new Constraints(240, 270));
    DoubleSupplier xSpeedSupplier;
    DoubleSupplier ySpeedSupplier;
    DoubleSupplier rotationSupplier;

    boolean isBlueAlliance = false;
    boolean isRobotCentric = false;
    Optional<Translation2d> aimTarget = Optional.empty();
    boolean isAimingAtReef = false;

    SwerveDrivePoseEstimator estimator;

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = CameraSource.DEFAULT_FIELD;

    public Drive(SwerveDriveMap map, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation,
            VisionMap visionMap) {

        super(new SwerveDriveData(), map);
        this.visionMap = visionMap;

        getMap().gyro.reset();
        kinematics = new SwerveDriveKinematics(map.frontLeft.getLocation(), map.frontRight.getLocation(),
                map.rearLeft.getLocation(), map.rearRight.getLocation());
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond;
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond;

        estimator = new SwerveDrivePoseEstimator(kinematics, getMap().gyro.getRotation2d(),
                getData().getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01));

        AutoBuilder.configure(estimator::getEstimatedPosition,
                this::setPose,
                this::getSpeeds,
                (speeds, feedforwards) -> move(speeds),
                map.holonomicDrive,
                map.config,
                () -> !isBlueAlliance,
                this);

        rotationPID.enableContinuousInput(-180, 180);

        this.xSpeedSupplier = xSpeed;
        this.ySpeedSupplier = ySpeed;
        this.rotationSupplier = rotation;

    }

    public void setPose(Pose2d pose) {
        estimator.resetPosition(getMap().gyro.getRotation2d(),
                getData().getModulePositions(), pose);
    }

    public Command setPoseCommand(Supplier<Pose2d> pose) {
        return runOnce(() -> {
            setPose(pose.get());
        });
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getData().getModuleStates());
    }

    public Command robotCentricDrive() {
        return startEnd(() -> {
            isRobotCentric = true;
        }, () -> {
            isRobotCentric = false;
        });
    }

    public Command aimAtReefCenter() {
        return startEnd(() -> {
            aimTarget = Optional.of(getReefCenter());
        }, () -> {
            aimTarget = Optional.empty();
        });
    }

    // public Command alignToReefBranch
    // Probably want arguments of leftBranch or rightBranch?
    // Pull reef coords from FieldConstants somehow
    // Find nearest reef apriltag (do we need a different method of estimation
    // rather than global? Might want to just focus on one tag rather than multiple
    // behind the robot. Reef tags are blue 17-22, red 6-11)
    // Use coords and move command to move to either left or right branch
    // (translation to get robotToReef and then move)
    // Need rotation in here somewhere. Logic from rotateTo command, since ideally
    // we move and rotate at the same time

    public enum Branch {
        leftBranch,
        rightBranch
    };

    // public Command alignToReefBranch(Branch leftBranch, Branch rightBranch) {

    // };

    public Command moveInDirection(double xSpeed, double ySpeed, double seconds) {
        return run(() -> {
            move(xSpeed, ySpeed, 0, false);
        }).withTimeout(seconds).andThen(safeStateCmd());
    }

    @Override
    public void reset() {
        Rotation2d heading = isBlueAlliance ? new Rotation2d() : new Rotation2d(Math.PI);
        Pose2d pose = estimator.getEstimatedPosition();
        setPose(new Pose2d(pose.getX(), pose.getY(), heading));
    }

    @Override
    public void safeState() {
        move(0.0, 0.0, 0.0, false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
        super.periodic();
        isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        estimator.update(getMap().gyro.getRotation2d(), getData().getModulePositions());

        visionMap.updateData(estimator);

        periodicMove(xSpeedSupplier.getAsDouble(), ySpeedSupplier.getAsDouble(), rotationSupplier.getAsDouble());

        Logger.recordOutput("Estimator Pose", estimator.getEstimatedPosition());
        Logger.recordOutput("Pose Angle", estimator.getEstimatedPosition().getRotation());
        Logger.recordOutput("Robot Rotation Gyro", getMap().gyro.getRotation2d());
    }

    private double calculateRotationSpeed(double targetAngleDegrees) {
        double estimatorAngle = estimator.getEstimatedPosition().getRotation().getDegrees();
        double rotationSpeed = rotationPID.calculate(estimatorAngle, targetAngleDegrees);
        rotationSpeed += Math.copySign(ROTATION_KS, rotationSpeed);
        // need to ensure we move at a fast enough speed for gyro to keep up
        if (Math.abs(rotationSpeed) < 0.02 || Math.abs(rotationPID.getPositionError()) < 0.75) {
            rotationSpeed = 0;
        }
        Logger.recordOutput("Target Angle", targetAngleDegrees);
        Logger.recordOutput("Estimator Angle", estimatorAngle);
        Logger.recordOutput("Rotation Speed", rotationSpeed);
        Logger.recordOutput("Target Velocity", rotationPID.getSetpoint().velocity);
        Logger.recordOutput("Position Error", rotationPID.getPositionError());
        return rotationSpeed;
    }

    private void periodicMove(final double xSpeed, final double ySpeed, final double rotation) {
        double rotationInput = DEADBAND.applyAsDouble(rotation);
        double xInput = DEADBAND.applyAsDouble(xSpeed);
        double yInput = DEADBAND.applyAsDouble(ySpeed);

        double translateXSpeed = xInput * maxDriveSpeedMetersPerSecond * SPEED_COEFFICIENT;
        double translateYSpeed = yInput * maxDriveSpeedMetersPerSecond * SPEED_COEFFICIENT;
        double rotationSpeed = rotationInput * maxRotationRadiansPerSecond * ROTATION_COEFFICIENT;
        if (aimTarget.isPresent()) {
            rotationSpeed = calculateRotateSpeedToTarget(aimTarget::get);
        }

        move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation, boolean robotCentricDrive) {
        ChassisSpeeds speeds;
        if (robotCentricDrive) {
            speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                    rotation, estimator.getEstimatedPosition().getRotation());
        }

        move(speeds);
    }

    private void move(final ChassisSpeeds speeds) {
        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);

        // All the states
        getData().setDesiredStates(moduleStates);
    }

    private Translation2d getRobotToTarget(Translation2d target) {
        return target.minus(estimator.getEstimatedPosition().getTranslation());
    }

    private double calculateRotateSpeedToTarget(Supplier<Translation2d> target) {
        var robotToTarget = getRobotToTarget(target.get());
        Logger.recordOutput("Target Pose", robotToTarget);
        return calculateRotationSpeed(robotToTarget.getAngle().getDegrees());
    }

    private Translation2d getReefCenter() {
        Translation3d translation;
        if (isBlueAlliance) {
            Translation3d poseLeft = kTagLayout.getTagPose(18).get().getTranslation();
            Translation3d poseRight = kTagLayout.getTagPose(21).get().getTranslation();
            Logger.recordOutput("Reef Center", "Blue");
            translation = poseLeft.plus(poseRight).div(2);
        } else {
            Translation3d poseLeft = kTagLayout.getTagPose(10).get().getTranslation();
            Translation3d poseRight = kTagLayout.getTagPose(7).get().getTranslation();
            Logger.recordOutput("Reef Center", "Red");
            translation = poseLeft.plus(poseRight).div(2);
        }
        return translation.toTranslation2d();
    }
}
