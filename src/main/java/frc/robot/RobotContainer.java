// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;


public class RobotContainer {


  public static final Pose2d BLUE_SCORE_POSE = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
  public static final Pose2d RED_SCORE_POSE = new Pose2d(13.5, 4.0, Rotation2d.fromDegrees(0));
  public static final Pose2d MID_FIELD = new Pose2d(8.5, 4.0, Rotation2d.fromDegrees(0));
  

  public final SwerveSubsystem drivebase = new SwerveSubsystem();




  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Swerve drive input stream using angular velocity for rotation
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> m_driverController.getLeftY() *-1,
              () -> m_driverController.getLeftX() * -1)
          .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
          .headingWhile(true);

  Command driveFeildOrientedDirectAngle = drivebase.driveFeildOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFeildOriented(driveAngularVelocity);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    // Build after the drivebase has configured AutoBuilder.
    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      // PathPlanner setup failed (check the console); don't crash, just offer no autos
      DriverStation.reportError("AutoBuilder is not configured, no autos available", false);
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("None (PathPlanner failed)", Commands.none());
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
    public static DriverStation.Alliance getAllianceColor() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue); 
  }
  private static Pose2d getScorePose() {
  if (getAllianceColor() == DriverStation.Alliance.Blue) {
    return BLUE_SCORE_POSE;
  } else{
    return RED_SCORE_POSE;
  }
  
}
  

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    m_driverController.x().and(m_driverController.b()).onTrue(drivebase.zeroGyro());
    m_driverController.rightTrigger().whileTrue(
        Commands.defer(
            () -> Commands.sequence(
                OTFPath.pathfindTo(MID_FIELD),
                OTFPath.pathfindTo(getScorePose())),
            java.util.Set.of(drivebase)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}