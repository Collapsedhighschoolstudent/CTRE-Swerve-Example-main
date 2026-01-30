// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Rotation;

//import org.opencv.features2d.GFTTDetector;

//import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants;

public class version extends SubsystemBase {
  private final Field2d field2d = new Field2d();
  private PoseEstimate botpose = new PoseEstimate();
  private Pose2d pose2d = new Pose2d();
  public static Pigeon2 PIGEON2 = new Pigeon2(0);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.32385, 0.32385),
      new Translation2d(0.32385, -0.32385),
      new Translation2d(-0.32385, 0.32385),
      new Translation2d(-0.32385, -0.32385));
  private SwerveModulePosition[] modulePositions = constants.modulePositions;
  public Rotation2d gyroAngle = new Rotation2d(0);
  public final Encoder drive_Encoder;
  public final Encoder turn_Encoder;
  private SwerveDrivePoseEstimator poseversion = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions,
      pose2d);

  public version() {
    SmartDashboard.putData("Field", field2d);
    SmartDashboard.putData("", field2d);
    drive_Encoder = new Encoder(0, 1);
    turn_Encoder = new Encoder(0, 1);

  }

  @Override
  public void periodic() {
    try {
      botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("LimeLight");
      field2d.getObject("botpose").setPose(botpose.pose);
    } catch (Exception e) {
      System.out.print(e);
    }

    LimelightHelpers.SetRobotOrientation("LimeLight", poseversion.getEstimatedPosition().getRotation().getDegrees(), 0,
        0, 0, 0, 0);
    gyroAngle = new Rotation2d(PIGEON2.getYaw().getValue());
    modulePositions = new SwerveModulePosition[] {
    getPosition()
    };
    poseversion.update(gyroAngle, modulePositions);

    try {
      poseversion.addVisionMeasurement(botpose.pose, botpose.timestampSeconds);
      pose2d = poseversion.getEstimatedPosition();
      field2d.setRobotPose(pose2d);
    } catch (Exception e) {
      System.out.print(e);
    }

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drive_Encoder.getDistance(), new Rotation2d(turn_Encoder.getDistance()));
  }
}
