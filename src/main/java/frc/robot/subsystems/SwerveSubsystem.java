// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  // Field widget showing where the robot thinks it is, the active auto path, and the raw camera pose.
  // Sim GUI / Glass: NetworkTables > SmartDashboard > Field
  private final Field2d field = new Field2d();


  // Must match the camera name set in the Limelight web UI
  private static final String LIMELIGHT = "limelight-swamp";

  // Leave false while the NavX is broken: vision then supplies the robot's heading (MegaTag1).
  // Set true once the gyro works: gyro supplies heading, vision only corrects x/y (MegaTag2).
  private static final boolean GYRO_WORKING = false;

  // Ignore tags farther than this (meters); far tags give noisy poses
  private static final double MAX_TAG_DIST_METERS = 4.0;

  public SwerveSubsystem() {
    try {
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(
                  Constants.maxSpeed,
                  new Pose2d(
                      new Translation2d(Meter.of(1), Meter.of(1)), Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // PathPlanner drives with chassis speeds, so YAGSL's own heading correction and
    // cosine compensation fight it. Turn them off.
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);

    SmartDashboard.putData("Field", field);

    // Must be last: AutoBuilder needs swerveDrive to exist.
    setupPathPlanner();
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Without this config AutoBuilder is never set up and no autos will load
      DriverStation.reportError("PathPlanner RobotConfig failed: " + e.getMessage(), e.getStackTrace());
      return;
    }

    AutoBuilder.configure(
        swerveDrive::getPose,
        swerveDrive::resetOdometry,
        swerveDrive::getRobotVelocity,
        (speedsRobotRelative, feedforwards) -> swerveDrive.setChassisSpeeds(speedsRobotRelative),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // translation PID
            new PIDConstants(5.0, 0.0, 0.0)), // rotation PID
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == Alliance.Red;
        },
        this);

    // Draw the path being followed and the point the robot is chasing on the field widget
    PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
    PathPlannerLogging.setLogTargetPoseCallback(pose -> field.getObject("targetPose").setPose(pose));
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  public double getMaxAngularVelocity() {
    return swerveDrive.getMaximumChassisAngularVelocity();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command driveFeildOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public Command zeroGyro() {
    return runOnce(() -> swerveDrive.zeroGyro());
  }

  private void updateVision() {
    // No camera in the sim
    if (RobotBase.isSimulation()) return;

    LimelightHelpers.PoseEstimate est;
    if (GYRO_WORKING) {
      // MegaTag2 needs the robot's heading from the gyro every loop
      LimelightHelpers.SetRobotOrientation(
          LIMELIGHT, swerveDrive.getYaw().getDegrees(), 0, 0, 0, 0, 0);
      est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);
    } else {
      // MegaTag1 works out position AND heading from the tags alone
      est = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT);
    }

    if (est == null || est.tagCount == 0) {
      SmartDashboard.putNumber("Vision/tagCount", 0);
      return;
    }
    SmartDashboard.putNumber("Vision/tagCount", est.tagCount);
    SmartDashboard.putNumber("Vision/avgTagDist", est.avgTagDist);

    if (est.avgTagDist > MAX_TAG_DIST_METERS) return;

    // Skip while spinning fast; camera data is unreliable then
    if (Math.abs(Math.toDegrees(swerveDrive.getRobotVelocity().omegaRadiansPerSecond)) > 720) return;

    double xyStdDev;
    double rotStdDev;
    if (GYRO_WORKING) {
      xyStdDev = 0.7;
      rotStdDev = 9999999; // ignore camera heading, trust the gyro
    } else if (est.tagCount >= 2) {
      xyStdDev = 0.5;
      rotStdDev = Units.degreesToRadians(10); // 2+ tags: heading is reliable
    } else {
      xyStdDev = 1.0;
      rotStdDev = Units.degreesToRadians(60); // 1 tag: heading can flip, trust it only a little
    }

    swerveDrive.addVisionMeasurement(
        est.pose, est.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

    // Raw camera pose, drawn separately so you can compare it to the fused pose
    field.getObject("vision").setPose(est.pose);
  }


  public Pose2d getPose() {
  return swerveDrive.getPose();
}

  @Override
  public void periodic() {
    updateVision();
    // Where the robot thinks it is (odometry fused with vision)
    field.setRobotPose(swerveDrive.getPose());
  }

  @Override
  public void simulationPeriodic() {}
}