// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;

import org.jumprobotics.util.EventSequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class AutoDock extends CommandBase {

  /*
   * How to approach docking
   * 1. Accelerate downwards very quickly. 
   * This will cause the robot to tip towards the station
   * This already works with the robot on the ground
   * With enough of an angular acceleration, this *could* work on an elevated surface
   * 2. Slam arm into station and raise robot on back wheels
   * Drive forward until wheels are on the station
   * Drive onto charge station and then implement (1)
   * 3. Slamming the arm into the charge station and just continuing to push
   * This will cause the robot to curl up and slip the other way
   * Threshold pitch is theoretically 45 deg
   * 
   * 
   */

  private Drivetrain dt;
  private Arm arm;
  private HashMap<Integer, EventSequence> path;
  /** Creates a new AutoDock. */
  public AutoDock(Drivetrain dt, Arm arm) {
    addRequirements(dt, arm);
    this.dt = dt;
    this.arm = arm;
    path.put(0, new EventSequence(this::isTipping, new InstantCommand(() -> arm.infiniteSpin())));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  private double readPitch(){
    return dt.getGyroIOInputs().pitchPosition;
  }

  private boolean isTipping(){
    return readPitch() > 45;//check that pitch is in degrees
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
