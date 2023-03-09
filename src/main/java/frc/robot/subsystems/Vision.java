// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private SwerveDrivePoseEstimator poseEstimator;
  

  private List<PhotonTrackedTarget> targets; 
  private AprilTagFieldLayout aprilTagFieldLayout;
  
  private RobotPoseEstimator robotPoseEstimator;
  private Pose2d currentPose2d = new Pose2d(0,0,new Rotation2d(0)); //this may need to be updater pre-emptively before the match
  private Double poseLatency = 0.0;

  //define all cameras here!
  private PhotonCamera limelightCamera = new PhotonCamera("OV5647");
  //private PhotonCamera cam;

  NetworkTableInstance instance = NetworkTableInstance.getDefault();
  NetworkTable table = instance.getTable("photonvision");

  /** Creates a new Vision. */
  public Vision() throws IOException{
    //cam = new PhotonCamera("camerea");

     //add all your cameras here!
     var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
     camList.add(new Pair<PhotonCamera, Transform3d>(limelightCamera, Constants.LIMELIGHT_POSITION));
     
     aprilTagFieldLayout  = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
     robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
     robotPoseEstimator.setLastPose(new Pose3d(0,0,0,new Rotation3d(0,0,0)));
  }

  @Override
  public void periodic() {
    try {

      Pair temp = getEstimatedGlobalPose(currentPose2d);
      currentPose2d = (Pose2d) temp.getFirst();
      poseLatency = (Double) temp.getSecond();

      } catch(NullPointerException e) {
      System.out.println("NullPointerException: no Apriltag fudicals in sight!");
      } catch(NoSuchElementException f) {
      System.out.println("NoSuchElementException: no Apriltag fudicals in sight!");
  }   

      SmartDashboard.putNumber("x (visual)", currentPose2d.getX());
      SmartDashboard.putNumber("y (visual)", currentPose2d.getY());
  }


  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
  }


  public Pose2d getVisualPose() {
    return currentPose2d;
  }

  public boolean fudicalsExist() {
    SmartDashboard.putBoolean("has targets", table.getEntry("hasTarget").getBoolean(false));
    return table.getEntry("hasTarget").getBoolean(false);
  }
}
