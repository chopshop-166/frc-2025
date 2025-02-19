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
import com.chopshop166.chopshoplib.maps.VisionMap.Data;
import com.chopshop166.chopshoplib.motors.Modifier;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.FieldConstants.Reef;
import frc.robot.Vision.Branch;

public class Drive extends LoggedSubsystem<SwerveDriveData, SwerveDriveMap> {

    public final SwerveDriveKinematics kinematics;
    private final VisionMap visionMap;
    private final VisionMap.Data visionData = new VisionMap.Data();
    private final Vision vision = new Vision();

    private final double maxDriveSpeedMetersPerSecond;
    private final double maxRotationRadiansPerSecond;
    private final double SPEED_COEFFICIENT = 1;
    private final double ROTATION_COEFFICIENT = 1;
    private final double ROTATION_KS = 0.19;
    final Modifier DEADBAND = Modifier.scalingDeadband(0.1);

    ProfiledPIDController rotationPID = new ProfiledPIDController(0.05, 0.0002, 0.000, new Constraints(240, 270));
    ProfiledPIDController translationPID_X = new ProfiledPIDController(0, 0, 0, new Constraints(1, 3));
    ProfiledPIDController translationPID_Y = new ProfiledPIDController(0, 0, 0, new Constraints(1, 3));
    DoubleSupplier xSpeedSupplier;
    DoubleSupplier ySpeedSupplier;
    DoubleSupplier rotationSupplier;

    boolean isBlueAlliance = false;
    boolean isRobotCentric = false;
    Optional<Translation2d> aimTarget = Optional.empty();
    boolean isAimingAtReef = false;
    Branch targetBranch = Branch.NONE;

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

        visionData.estimator = estimator;

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
            aimTarget = Optional.of(Vision.getReefCenter(isBlueAlliance));
        }, () -> {
            aimTarget = Optional.empty();
        });
    }

    // Need vision command that takes all info about reef tags and branches
    // (filterReefTags, pickBestReefLocation, adjustTranslationForBranch)

    public Command alignToReefBranch(Branch branch) {
        return run(() -> {
            vision.filterReefTags(isBlueAlliance, visionData.targets);
            Transform2d robotToBranch = vision
                    .adjustTranslationForBranch(vision.pickBestReefLocation(visionData.targets), branch);
            double XSpeed = translationPID_X.calculate(robotToBranch.getX());
            double YSpeed = translationPID_Y.calculate(robotToBranch.getY());
            double rotationSpeed = rotationPID.calculate(robotToBranch.getRotation().getDegrees());
            move(XSpeed, YSpeed, rotationSpeed, true);
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

        visionMap.updateData(visionData);

        periodicMove(xSpeedSupplier.getAsDouble(), ySpeedSupplier.getAsDouble(), rotationSupplier.getAsDouble());

        Logger.recordOutput("Estimator Pose", estimator.getEstimatedPosition());
        Logger.recordOutput("Pose Angle", estimator.getEstimatedPosition().getRotation());
        Logger.recordOutput("Robot Rotation Gyro", getMap().gyro.getRotation2d());
        Logger.recordOutput("Target Branch", targetBranch);

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
        if (targetBranch != Branch.NONE) {
            vision.filterReefTags(isBlueAlliance, visionData.targets);
            Transform2d reefLocation = vision.pickBestReefLocation(visionData.targets);
            Transform2d robotToBranch = vision
                    .adjustTranslationForBranch(reefLocation, targetBranch);
            Logger.recordOutput("Reef Location Transform2d", estimator.getEstimatedPosition().plus(reefLocation));
            Logger.recordOutput("Robot To Branch Transform2d", estimator.getEstimatedPosition().plus(robotToBranch));
            Pose2d robotPose = estimator.getEstimatedPosition();
            translateXSpeed = translationPID_X.calculate(robotPose.getX(), robotPose.getX() + robotToBranch.getX());
            translateYSpeed = translationPID_Y.calculate(robotPose.getY(), robotPose.getY() + robotToBranch.getY());
            rotationSpeed = rotationPID.calculate(robotPose.getRotation().getDegrees(),
                    robotPose.getRotation().getDegrees() - robotToBranch.getRotation().getDegrees());
            Logger.recordOutput("Robot To Branch Angle", robotToBranch.getRotation().getDegrees());
        }

        move(translateXSpeed, translateYSpeed, rotationSpeed, isRobotCentric);
    }

    public Command moveToBranch(Branch targetBranch) {
        return startEnd(() -> {
            this.targetBranch = targetBranch;
            isRobotCentric = true;

        }, () -> {
            this.targetBranch = Branch.NONE;
            isRobotCentric = false;
        });
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

}
