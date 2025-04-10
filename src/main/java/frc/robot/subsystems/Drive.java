package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
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
    private final double ROTATION_KS = 0.1;
    private final double DRIVE_KS = 0.1;
    final Modifier DEADBAND = Modifier.scalingDeadband(0.1);

    // ProfiledPIDController rotationPID = new ProfiledPIDController(0.06, 0.0002,
    // 0.000, new Constraints(240, 270));
    // ProfiledPIDController translationPID_X = new ProfiledPIDController(2.0, 0,
    // 0.0, new Constraints(2.5, 3.0));
    // ProfiledPIDController translationPID_Y = new ProfiledPIDController(2.0, 0,
    // 0.0, new Constraints(2.5, 3.0));

    ProfiledPIDController rotationPID = new ProfiledPIDController(0.06, 0.0002, 0.000, new Constraints(240, 270));
    ProfiledPIDController translationPID_X = new ProfiledPIDController(1.6, 0, 0.0, new Constraints(2.0, 3.0));
    ProfiledPIDController translationPID_Y = new ProfiledPIDController(1.6, 0, 0.0, new Constraints(2.0, 3.0));
    DoubleSupplier xSpeedSupplier;
    DoubleSupplier ySpeedSupplier;
    DoubleSupplier rotationSupplier;

    boolean isBlueAlliance = false;
    boolean isRobotCentric = false;
    Branch targetBranch = Branch.NONE;
    Pose2d targetPose = new Pose2d();

    ChassisSpeeds fielRelativeChassisSpeeds = new ChassisSpeeds();

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

        translationPID_X.setTolerance(0.035);
        translationPID_Y.setTolerance(0.035);
        rotationPID.setTolerance(2);
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

        fielRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(getData().getModuleStates()),
                estimator.getEstimatedPosition().getRotation());

        Logger.recordOutput("Drive/Estimator Pose", estimator.getEstimatedPosition());
        Logger.recordOutput("Drive/Robot Rotation Gyro", getMap().gyro.getRotation2d());
        Logger.recordOutput("Drive/Target Branch", targetBranch);
        Logger.recordOutput("Drive/Translation_X_PID/Error", translationPID_X.getPositionError());
        Logger.recordOutput("Drive/Translation_X_PID/Velocity", translationPID_X.getSetpoint().velocity);
        Logger.recordOutput("Drive/Translation_X_PID/At Goal", translationPID_X.atGoal());
        Logger.recordOutput("Drive/Translation_Y_PID/Error", translationPID_Y.getPositionError());
        Logger.recordOutput("Drive/Translation_Y_PID/Velocity", translationPID_Y.getSetpoint().velocity);
        Logger.recordOutput("Drive/Translation_Y_PID/At Goal", translationPID_Y.atGoal());
        Logger.recordOutput("Drive/Rotation_PID/Error", rotationPID.getPositionError());
        Logger.recordOutput("Drive/Rotation_PID/Velocity", rotationPID.getSetpoint().velocity);
        Logger.recordOutput("Drive/Rotation_PID/At Goal", rotationPID.atGoal());
        Logger.recordOutput("Drive/ActualChassisSpeeds",
                fielRelativeChassisSpeeds);
    }

    private void visionCalcs() {
        Optional<Integer> closestReefTag = Optional.empty();
        // if (visionData.targets.size() > 0) {
        // vision.filterReefTags(isBlueAlliance, visionData.targets);
        // PhotonTrackedTarget reefToRobot =
        // vision.pickBestReefLocation(visionData.targets);
        // closestReefTag = Optional.of(reefToRobot.fiducialId);

        // } else {
        // Fall back to picking tag based on global pose
        closestReefTag = Optional.of(vision.findNearestTagId(isBlueAlliance, estimator));
        // }

        if (closestReefTag.isPresent()) {
            var chosenTagPoseOption = kTagLayout.getTagPose(closestReefTag.get());
            if (chosenTagPoseOption.isPresent()) {
                Pose2d chosenTagPose = chosenTagPoseOption.get().toPose2d();
                Logger.recordOutput("Drive/Chosen Tag", chosenTagPose);
                chosenTagPose = chosenTagPose.transformBy(targetBranch.getOffset());
                Logger.recordOutput("Drive/Chosen Tag Branch Offset", chosenTagPose);
                targetPose = chosenTagPose;
            }
        }
    }

    private void periodicMove(final double xSpeed, final double ySpeed, final double rotation) {
        double rotationInput = DEADBAND.applyAsDouble(rotation);
        double xInput = DEADBAND.applyAsDouble(xSpeed);
        double yInput = DEADBAND.applyAsDouble(ySpeed);

        double translateXSpeedMPS = xInput * maxDriveSpeedMetersPerSecond * SPEED_COEFFICIENT;
        double translateYSpeedMPS = yInput * maxDriveSpeedMetersPerSecond * SPEED_COEFFICIENT;
        double rotationSpeed = rotationInput * maxRotationRadiansPerSecond * ROTATION_COEFFICIENT;

        if (targetBranch != Branch.NONE) {

            visionCalcs();
            Pose2d robotPose = estimator.getEstimatedPosition();
            // soooooooooo x and y are backwards somehow. Values underneath are correct
            translateYSpeedMPS = translationPID_X.calculate(robotPose.getX(), targetPose.getX());
            translateYSpeedMPS += Math.copySign(DRIVE_KS, translateYSpeedMPS);
            translateXSpeedMPS = translationPID_Y.calculate(robotPose.getY(), targetPose.getY());
            translateXSpeedMPS += Math.copySign(DRIVE_KS, translateXSpeedMPS);
            // Direction is swapped on Red side so need to negate PID output
            if (!isBlueAlliance) {
                translateXSpeedMPS *= -1;
                translateYSpeedMPS *= -1;
            }
            rotationSpeed = rotationPID.calculate(robotPose.getRotation().getDegrees(),
                    targetPose.getRotation().getDegrees());
            rotationSpeed += Math.copySign(ROTATION_KS, rotationSpeed);
        }

        move(translateXSpeedMPS, translateYSpeedMPS, rotationSpeed, isRobotCentric);
    }

    public Command moveToBranch(Branch targetBranch) {
        return startEnd(() -> {
            translationPID_X.reset(estimator.getEstimatedPosition().getX(),
                    fielRelativeChassisSpeeds.vxMetersPerSecond);
            translationPID_Y.reset(estimator.getEstimatedPosition().getY(),
                    fielRelativeChassisSpeeds.vyMetersPerSecond);
            rotationPID.reset(new State(estimator.getEstimatedPosition().getRotation().getDegrees(), 0));
            this.targetBranch = targetBranch;
            isRobotCentric = false;

        }, () -> {
            this.targetBranch = Branch.NONE;
        });
    }

    public Command moveToBranchWait(Branch targetBranch) {
        return runOnce(() -> {
            translationPID_X.reset(estimator.getEstimatedPosition().getX(),
                    fielRelativeChassisSpeeds.vxMetersPerSecond);
            translationPID_Y.reset(estimator.getEstimatedPosition().getY(),
                    fielRelativeChassisSpeeds.vyMetersPerSecond);
            rotationPID.reset(new State(estimator.getEstimatedPosition().getRotation().getDegrees(), 0));
            this.targetBranch = targetBranch;
        }).andThen(run(() -> {
        })).until(() -> {
            return translationPID_X.atGoal() && translationPID_Y.atGoal() && rotationPID.atGoal();
        }).finallyDo(() -> {
            this.targetBranch = Branch.NONE;
        });

    }

    public BooleanSupplier visionPIDTrue() {
        return () -> {
            return translationPID_X.atGoal() && translationPID_Y.atGoal() && rotationPID.atGoal();
        };
    }

    private void move(final double xSpeed, final double ySpeed,
            final double rotation, boolean robotCentricDrive) {
        ChassisSpeeds speeds;
        if (robotCentricDrive) {
            speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed,
                    rotation, isBlueAlliance ? estimator.getEstimatedPosition().getRotation()
                            : estimator.getEstimatedPosition().getRotation()
                                    .plus(new Rotation2d(Units.degreesToRadians(180))));
        }
        Logger.recordOutput("Drive/TargetChassisSpeeds", speeds);
        move(speeds);
    }

    private void move(final ChassisSpeeds speeds) {
        // Now use this in our kinematics
        final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxDriveSpeedMetersPerSecond);

        // All the states
        getData().setDesiredStates(moduleStates);
    }

}
