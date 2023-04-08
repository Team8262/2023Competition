// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class HoldArm extends CommandBase {
  /** Creates a new HoldArm. */
  private double posLow, posHigh;
  private double positionTolerance =  0.01;
  private Arm arm;
  private double seconds;
  private double elapsedTime;
  private double startTime;
  private double elapsedSeconds;
  public HoldArm(Arm arm, double posLow, double posHigh, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.posLow = posLow;
    this.posHigh = posHigh;
    this.arm = arm;
    this.seconds = seconds;
    this.startTime = System.currentTimeMillis();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.gayAnglesRaw(posLow, posHigh, 1);
    elapsedTime = System.currentTimeMillis() - startTime;
    elapsedSeconds = elapsedTime / 1000;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsedSeconds >= seconds;
  }
}