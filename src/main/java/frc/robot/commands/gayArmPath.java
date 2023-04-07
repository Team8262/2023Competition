// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;

public class gayArmPath extends CommandBase {
  /** Creates a new FollowArmPat. */
  private Arm arm;
  private int path;
  private int step;
  private Joystick stik = new Joystick(1);
  double isCone = stik.getRawAxis(3);
  boolean coneBool = isCone < 0;
  double[][] pathh;
  private final double positionTolerance = 0.06; //This is in radians i think
  private double speed;
  public gayArmPath(Arm arm, int path, double speed) {
    addRequirements(arm);
    this.arm = arm;
    this.path = coneBool ? path*2 : path;
    this.speed = speed;
    pathh = RobotContainer.armPaths.get(path);
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

   arm.gayAnglesRaw(pathh[step][0], pathh[step][1], speed);
    if(atPoint(pathh[step][0], pathh[step][1]) && step != pathh.length - 1){
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