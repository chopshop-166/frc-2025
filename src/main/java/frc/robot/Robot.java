// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;
import frc.robot.subsystems.AlgaeDestage;
import frc.robot.subsystems.CoralManip;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Outtake;

public final class Robot extends CommandRobot {

    private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    // Helpers
    final DoubleUnaryOperator driveScaler = getScaler(0.45, 0.25);

    private Drive drive = new Drive(map.getDriveMap(), () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftX());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftY());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getRightX());
    }, map.getVisionMap());
    private Led led = new Led(map.getLedMap());
    private AlgaeDestage algaeDestage = new AlgaeDestage(map.getAlgaeDestageMap());
    private Outtake outtake = new Outtake(map.getOuttakeMap());
    private CoralManip coralManip = new CoralManip(map.getCoralManipMap());
    private Elevator elevator = new Elevator(map.getElevatorMap());
    private DeepClimb deepClimb = new DeepClimb(map.getDeepClimbMap());

    private CommandSequences commandSequences = new CommandSequences(drive, led, algaeDestage, coralManip, elevator);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    public void registerNamedCommands() {

        NamedCommands.registerCommand("Intake Game Piece", commandSequences.intake());
        NamedCommands.registerCommand("Score Coral L1", commandSequences.scoreL1Auto());
        NamedCommands.registerCommand("Score Coral L2", commandSequences.scoreCoralAuto(ElevatorPresets.SCOREL2));
        NamedCommands.registerCommand("Score Coral L3", commandSequences.scoreCoralAuto(ElevatorPresets.SCOREL3));
    }

    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    private final SendableChooser<Command> autoChooser;

    public Robot() {
        super();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
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

        map.setupLogging();

        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values
        // may
        // be added.
        Logger.start();

        led.colorAlliance().schedule();
        DriverStation.silenceJoystickConnectionWarning(true);

    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        led.colorAlliance().schedule();
    }

    @Override
    public void configureButtonBindings() {
        driveController.back().onTrue(commandSequences.resetAll());
        driveController.leftBumper()
                .whileTrue(drive.robotCentricDrive());

        copilotController.a().onTrue(commandSequences.intake());

        copilotController.x().whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL1))
                .onFalse(commandSequences.scoreL1());

        copilotController.b().whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL2))
                .onFalse(commandSequences.score());

        copilotController.y().whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL3))
                .onFalse(commandSequences.score());

        driveController.rightBumper().whileTrue(drive.aimAtReefCenter());

    }

    @Override
    public void populateDashboard() {

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
        elevator.setDefaultCommand(elevator.move(RobotUtils.deadbandAxis(.1, () -> -copilotController.getLeftY())));
        deepClimb.setDefaultCommand(deepClimb.rotate(RobotUtils.deadbandAxis(0.1, () -> copilotController.getRightY())));
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