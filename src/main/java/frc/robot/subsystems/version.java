// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry .Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Translation2d;

//import org.opencv.features2d.GFTTDetector;

//import com.ctre.phoenix6.swerve.SwerveDrivetrain;992

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;



public class version extends SubsystemBase {
  private final Field2d field2d = new Field2d();
  private PoseEstimate botpose = new PoseEstimate();
  private Pose2d pose2d = new Pose2d();
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.32385, 0.32385),
      new Translation2d(0.32385, -0.32385),
      new Translation2d(-0.32385, 0.32385),
      new Translation2d(-0.32385, -0.32385)
  );
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
};
  private  Rotation2d gyroAngle = new Rotation2d(0);
  private final SwerveDrivePoseEstimator poseversion = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, pose2d);

  
  public version() {
      SmartDashboard.putData("Field", field2d);
      SmartDashboard.putData("", field2d);
  }

  @Override
  public void periodic() {
    try{  botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("LimeLight");
    field2d.getObject("botpose").setPose(botpose.pose);}
    catch (Exception e){
      System.out.print(e);
    }
  
    poseversion.update(gyroAngle, modulePositions);
    LimelightHelpers.SetRobotOrientation("Limelight", poseversion.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    try{poseversion.addVisionMeasurement(botpose.pose, botpose.timestampSeconds);
    pose2d = poseversion.getEstimatedPosition();
    field2d.setRobotPose(pose2d);}
    catch (Exception e){
      System.out.print(e);
    }
    
  }
}
