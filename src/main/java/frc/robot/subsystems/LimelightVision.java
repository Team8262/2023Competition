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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightVision extends SubsystemBase {

  public boolean DEBUGGING = true;

  public NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry pipeline;

  double x = 0;
  double y = 0;
  double area = 0;

  /** Creates a new Vision. */
  public LimelightVision() throws IOException{

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    pipeline = table.getEntry("pi peline");

     //add all your cameras here! 
    
     if(DEBUGGING) {
      ShuffleboardTab Vision = Shuffleboard.getTab("vision");
      Vision.addBoolean("Has targets", () -> {if(tv.getInteger(0) == 1) { return true;} else {return false;}});
      Vision.addDouble("target x", () -> tx.getDouble(0));
      Vision.addDouble("target y", () -> ty.getDouble(0));
      Vision.addDouble("target area", () -> ta.getDouble(0));
     }
  }

  @Override
  public void periodic() {
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    System.out.println(x);
  }

  public double getTargetX() {
    return x;
  }

  public double getTargetY() {
    return y;
  }

  public boolean fudicalsExist() {
    if(tv.getInteger(0) == 1) {
       return true;
    } else {
      return false;
    }
  }

  /*
   * (may need to be changed/set properly)
   * Pipeline IDs:
   * 0 - reflective tape
   * 1 - apriltags
   */
  public void setPipelineIndex(int i) {
    pipeline.setInteger(i);
  }
}
