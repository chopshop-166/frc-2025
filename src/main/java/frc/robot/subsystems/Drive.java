package frc.robot.subsystems;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

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
    Map<Integer, Pose2d> BLUE_APRIL_TAGS_REEF_POSITIONS = new HashMap<>();
    Map<Integer, Pose2d> RED_APRIL_TAGS_REEF_POSITIONS = new HashMap<>();

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

        for (int i = 6; i < 11; i++) {
            final int tmpI = i;
            kTagLayout.getTagPose(i).ifPresent(pose -> {
                RED_APRIL_TAGS_REEF_POSITIONS.put(tmpI, pose.toPose2d());
            });
        }

        for (int i = 17; i < 22; i++) {
            final int tmpI = i;
            kTagLayout.getTagPose(i).ifPresent(pose -> {
                BLUE_APRIL_TAGS_REEF_POSITIONS.put(tmpI, pose.toPose2d());
            });
        }

    }

    public enum Branch {
        leftBranch,
        rightBranch
    };

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

    // Function that takes current robot pose and finds the nearest reef apriltag to
    // it, returning the id
    /////////////////////// After --> and taking the average pose from both cameras?
    ///
    // align to reef branch command part two:
    // 1. Filter list of apriltags and figure out which ones we see
    // 2. filter down to reef apriltags and get poses
    // 3. find best target (nearest tag or tag closest to center of sight? Another
    // way?)
    // 4. now we know that best target plane is a reef side. Apriltags are always in
    // the middle at the same point in relation to the reef side, so then
    // apply translation offset for left or right branch
    // 5. With branch pose, use autoBuilder to maneuver to the branch (angle
    // funky??)

    //

    public boolean filterReefTags() {
        boolean targetVisible = false;
        for (CameraSource source : visionMap.visionSources) {
            var results = source.camera.getAllUnreadResults();

            if (!results.isEmpty()) {
                PhotonPipelineResult visionResult = results.get(results.size() - 1);
                // At least one AprilTag was seen by the camera
                for (var target : visionResult.targets) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetVisible = true;
                    }
                }
            }
        }
        return targetVisible;
    }

    // public Pose2d getVisibleReefTags() {
    // var targets = visionMap.visionSources.getLatestResult().getTargets();
    // for (var tgt : targets) {
    // var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
    // if (tagPose.isEmpty())
    // continue;
    // numTags++;
    // avgDist +=
    // tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    // }
    // }

    public int findNearestTagId() {
        if (isBlueAlliance) {
            return getNearestTagIdImpl(BLUE_APRIL_TAGS_REEF_POSITIONS);
        } else {
            return getNearestTagIdImpl(RED_APRIL_TAGS_REEF_POSITIONS);
        }
    }

    public Command alignToReefBranch(Branch branch) {
        return run(() -> {

        });
    }

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

        // visionMap.updateData(estimator);

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

    private int getNearestTagIdImpl(Map<Integer, Pose2d> poses) {
        Pose2d robotPose = estimator.getEstimatedPosition();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();
        return Collections.min(
                poses.entrySet(),
                Comparator.comparing((Map.Entry<Integer, Pose2d> entry) -> {
                    return robotTranslation.getDistance(entry.getValue().getTranslation());
                }).thenComparing((Map.Entry<Integer, Pose2d> entry) -> {
                    return Math.abs(robotRotation.minus(entry.getValue().getRotation()).getRadians());
                })).getKey();
    }
}
