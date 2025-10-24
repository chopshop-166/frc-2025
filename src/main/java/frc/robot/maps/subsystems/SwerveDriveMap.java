package frc.robot.maps.subsystems;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import yams.mechanisms.config.SwerveDriveConfig;

public record SwerveDriveMap(SwerveDriveConfig swerveDriveConfig, RobotConfig pathPlannerConfig,
        PPHolonomicDriveController holonomicDrive) {

}
