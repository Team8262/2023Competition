// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class gayArmPath extends CommandBase {
  /** Creates a new FollowArmPat. */
  private Arm arm;
  private double[][] path;
  private int step;
  private final double positionTolerance = 0.06; //This is in radians i think
  private double speed;
  public gayArmPath(Arm arm, double[][] path, double speed) {
    addRequirements(arm);
    this.arm = arm;
    this.path = path;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arm.gayAnglesRaw(0, -.2);
    //arm.spin();

   arm.gayAnglesRaw(path[step][0], path[step][1], speed);
    if(atPoint(path[step][0], path[step][1]) && step != path.length - 1){
      step++;
      System.out.println(step);
   }
   
  }

  private boolean atPoint(double basePos, double armPos){
    double[] current = arm.getRealAngle();
    return Math.abs(current[0] - basePos) < positionTolerance && Math.abs(current[1] - armPos) < positionTolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // arm.diee();
    System.out.println("Did it");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //THis should be removed before comp
    /*
    if(step == path.length) {
      arm.diee();
    }
    return step == path.length;
    */
    return false; //step == path.length;
    //WES
  }
}