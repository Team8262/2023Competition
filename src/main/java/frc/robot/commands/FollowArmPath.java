// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FollowArmPath extends CommandBase {
  /** Creates a new FollowArmPat. */
  private Arm arm;
  private double[][] path;
  private int step;
  private final double positionTolerance = 0.1; //This is in radians i think
  public FollowArmPath(Arm arm, double[][] path) {
    addRequirements(arm);
    this.arm = arm;
    this.path = path;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setAngles(path[step][0], path[step][1]);
    if(atPoint(path[step][0], path[step][1])){
      step++;
    }
  }

  private boolean atPoint(double basePos, double armPos){
    double[] current = arm.getCurrentPositions();

    return Math.abs(current[0] - basePos) < positionTolerance && Math.abs(current[1] - armPos) < positionTolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return step == path.length;
  }
}