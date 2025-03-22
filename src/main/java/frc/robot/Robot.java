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
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Vision.Branch;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmRotatePresets;
import frc.robot.maps.subsystems.ElevatorMap.ElevatorPresets;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.CoralManip;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Led;

public final class Robot extends CommandRobot {

    private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);
    private Trigger elevatorSafeTrigger;

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
    private CoralManip coralManip = new CoralManip(map.getCoralManipMap());
    private Elevator elevator = new Elevator(map.getElevatorMap(),
            RobotUtils.deadbandAxis(.15, () -> -copilotController.getLeftY()));
    private DeepClimb deepClimb = new DeepClimb(map.getDeepClimbMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap());
    private Funnel funnel = new Funnel(map.getFunnelMap());

    private CommandSequences commandSequences = new CommandSequences(drive, led, coralManip, elevator,
            armRotate, funnel, deepClimb);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    public void registerNamedCommands() {

        NamedCommands.registerCommand("Intake Game Piece", commandSequences.intake());
        NamedCommands.registerCommand("Wait Until Game Piece", coralManip.waitUntilGamePiece());
        NamedCommands.registerCommand("Position Coral L1",
                commandSequences.moveElevator(ElevatorPresets.SCOREL1, ArmRotatePresets.SCOREL1));
        NamedCommands.registerCommand("Position Coral L2",
                commandSequences.moveElevator(ElevatorPresets.SCOREL2, ArmRotatePresets.SCOREL2));
        NamedCommands.registerCommand("Position Coral L3",
                commandSequences.moveElevator(ElevatorPresets.SCOREL3, ArmRotatePresets.SCOREL3));
        NamedCommands.registerCommand("Position Coral L4",
                commandSequences.moveElevator(ElevatorPresets.SCOREL4, ArmRotatePresets.SCOREL4));
        NamedCommands.registerCommand("Score Coral", coralManip.score());
        NamedCommands.registerCommand("Stow",
                commandSequences.moveElevator(ElevatorPresets.STOW, ArmRotatePresets.STOW));
        NamedCommands.registerCommand("Zero Da Elevatah", elevator.zero());
        NamedCommands.registerCommand("Elevator to intake", elevator.moveTo(ElevatorPresets.STOW));
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
        elevatorSafeTrigger = new Trigger(elevator.elevatorSafeTrigger());
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
        driveController.back().onTrue(drive.resetCmd());
        driveController.a()
                .whileTrue(drive.robotCentricDrive());
        driveController.rightBumper().whileTrue(drive.moveToBranch(Branch.RIGHT_BRANCH));
        driveController.leftBumper().whileTrue(drive.moveToBranch(Branch.LEFT_BRANCH));

        elevatorSafeTrigger.onTrue(commandSequences.intakeBottom());

        driveController.x().onTrue(funnel.rotateForward());
        driveController.y().onTrue(funnel.rotateBackward());

        copilotController.a().onTrue(commandSequences.intake());
        copilotController.b()
                .whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL2, ArmRotatePresets.SCOREL2))
                .onFalse(coralManip.score());
        copilotController.x()
                .whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL1, ArmRotatePresets.SCOREL1))
                .onFalse(coralManip.scoreL1());
        copilotController.y()
                .whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL3, ArmRotatePresets.SCOREL3))
                .onFalse(coralManip.score());

        copilotController.back().onTrue(commandSequences.resetCopilot());
        copilotController.start().onTrue(elevator.zero());

        copilotController.getPovButton(POVDirection.RIGHT).onTrue(coralManip.feedAlgae());
        copilotController.getPovButton(POVDirection.DOWN).whileTrue(coralManip.feed());

        copilotController.getPovButton(POVDirection.LEFT)
                .onTrue(commandSequences.moveElevator(ElevatorPresets.ALGAEL2, ArmRotatePresets.ALGAE)
                        .alongWith(coralManip.feedAlgae()));

        copilotController.getPovButton(POVDirection.UP)
                .onTrue(commandSequences.moveElevator(ElevatorPresets.ALGAEL3, ArmRotatePresets.ALGAE)
                        .alongWith(coralManip.feedAlgae()));
        // copilotController.leftBumper().whileTrue(armRotate.moveTo(ArmRotatePresets.OUT));
        copilotController.rightBumper()
                .whileTrue(commandSequences.moveElevator(ElevatorPresets.SCOREL4, ArmRotatePresets.SCOREL4))
                .onFalse(coralManip.score().andThen(armRotate.moveTo(ArmRotatePresets.OUT)));
        copilotController.leftBumper().whileTrue(deepClimb.spoolIn());
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
        armRotate.setDefaultCommand(armRotate.move(RobotUtils.deadbandAxis(.1, () -> -copilotController.getRightY())));
        // funnel.setDefaultCommand(
        // funnel.move(RobotUtils.deadbandAxis(.1, () ->
        // -copilotController.getLeftTriggerAxis())));
        deepClimb
                .setDefaultCommand(
                        deepClimb.rotate(RobotUtils.deadbandAxis(0.1, () -> copilotController.getTriggers())));

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