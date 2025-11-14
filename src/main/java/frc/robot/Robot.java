// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Vision.Branch;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.UndertakerMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Speeds;
import frc.robot.subsystems.Undertaker;

public final class Robot extends CommandRobot {

    private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);
    private Trigger visionPIDTrigger;

    // Helpers
    final DoubleUnaryOperator driveScaler = getScaler(0.45, 0.25);

    private Drive drive = new Drive(map.getDriveMap(), () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftX());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftY());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getRightX());
    }, map.getVisionMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap(),
            RobotUtils.deadbandAxis(.1, () -> (copilotController.getLeftY())));
    private Shooter shooter = new Shooter(map.getShooterMap());
    private Undertaker undertaker = new Undertaker(map.getUndertakerMap());
    private Intake intake = new Intake(map.getIntakeMap());
    private CommandSequences commandSequences = new CommandSequences(drive, intake, shooter, armRotate, undertaker);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Align to Left Branch", drive.moveToBranchWait(Branch.LEFT_BRANCH));
        NamedCommands.registerCommand("Align to Right Branch", drive.moveToBranchWait(Branch.RIGHT_BRANCH));
    }

    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    private final SendableChooser<Command> autoChooser;

    public Robot() {
        super();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        visionPIDTrigger = new Trigger(drive.visionPIDTrue());
    }

    @Override
    public void robotInit() {
        super.robotInit();

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        Logger.recordMetadata("RobotMap", map.getClass().getName());

        map.setupLogging();

        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values
        // may
        // be added.
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            Logger.recordOutput("Drive/PathPlannerTargetPose", pose);
        });

        CommandScheduler.getInstance().onCommandInterrupt((oldCmd, newCmd) -> {
            if (!DriverStation.isFMSAttached()) {
                String newSub = "<NONE>";
                String newName = "<NONE>";
                if (newCmd.isPresent()) {
                    newSub = newCmd.get().getSubsystem();
                    newName = newCmd.get().getName();
                }
                System.out.println("Command interrupt: `" + oldCmd.getSubsystem() + "/" + oldCmd.getName() +
                        "` -> `" + newSub + "/" + newName + "`");
            }
        });

    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void configureButtonBindings() {
        driveController.back().onTrue(drive.resetCmd());
        driveController.a()
                .whileTrue(drive.robotCentricDrive());
        driveController.x().whileTrue(intake.spinOut().alongWith(undertaker.spinOut()));
        driveController.b()
                .whileTrue(commandSequences.charge(Speeds.SUBWOOFER_SHOT, ArmRotatePresets.SHOOT_HIGH))
                .onFalse(commandSequences.release());
        driveController.y().whileTrue(commandSequences.charge(Speeds.SHUTTLE_SHOT, ArmRotatePresets.SHOOT_LOW))
                .onFalse(commandSequences.release());
        copilotController.a().onTrue(commandSequences.moveAndIntake());

        copilotController.back().onTrue(intake.safeStateCmd().andThen(armRotate.safeStateCmd()));
        copilotController.start().onTrue(shooter.setSpeed(Speeds.OFF));
        copilotController.a().onTrue(commandSequences.moveAndIntake());
        copilotController.b()
                .whileTrue(commandSequences.charge(Speeds.SUBWOOFER_SHOT, ArmRotatePresets.SHOOT_HIGH))
                .onFalse(commandSequences.release());
        copilotController.y().whileTrue(commandSequences.charge(Speeds.SHUTTLE_SHOT, ArmRotatePresets.SHOOT_LOW))
                .onFalse(commandSequences.release());
        copilotController.x().whileTrue(intake.spinOut().alongWith(undertaker.spinOut()));
        copilotController.rightStick().whileTrue(intake.feedShooter());
        copilotController.povDown().onTrue(commandSequences.shooterSpeed(Speeds.FULL_SPEED));
    }

    @Override
    public void populateDashboard() {
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    @Override
    public void setDefaultCommands() {
    }

    public DoubleUnaryOperator getScaler(double leftRange, double rightRange) {
        return speed -> {
            double leftTrigger = driveController.getLeftTriggerAxis();
            double rightTrigger = driveController.getRightTriggerAxis();
            double modifier = (rightRange * rightTrigger) - (leftRange * leftTrigger) + 0.75;
            return modifier * speed;
        };
    }
}