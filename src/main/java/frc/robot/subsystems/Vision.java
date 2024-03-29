// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import javax.swing.DebugGraphics;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  public boolean DEBUGGING = true;

  //define all cameras here!
  private PhotonCamera limelightCamera = new PhotonCamera("OV5647");

  /** Creates a new Vision. */
  public Vision() throws IOException{

     //add all your cameras here!
     var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
     camList.add(new Pair<PhotonCamera, Transform3d>(limelightCamera, Constants.LIMELIGHT_POSITION));
     limelightCamera.setPipelineIndex(0);  

     if(DEBUGGING) {
      ShuffleboardTab Vision = Shuffleboard.getTab("vision");
      Vision.addBoolean("Has targets", () -> limelightCamera.getLatestResult().hasTargets());
     }

  }

  @Override
  public void periodic() {

  }

  public boolean fudicalsExist() {
    PhotonPipelineResult result = limelightCamera.getLatestResult();
    return result.hasTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    return limelightCamera.getLatestResult().getBestTarget();
  }

  /*
   * (may need to be changed/set properly)
   * Pipeline IDs:
   * 0 - reference
   * 1 - reflective tape
   * 2 - apriltags
   */
  public void setPipelineIndex(int i) {
    limelightCamera.setPipelineIndex(i);
  }
}
