// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.End;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

public class IntakeThings extends CommandBase {
  /** Creates a new IntakeThings. */
  private Intake intake;
  private End end;
  public IntakeThings(Intake intake, End end) {

    addRequirements(intake, end);
    this.intake = intake;
    this.end = end;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intaked()) {
      intake.setSpeed(0);
      end.setSpeed(0);
    } else {
      intake.setSpeed(1);
      end.setSpeed(1);

    }
  }

  private boolean intaked() {
    return end.getTop().getOutputCurrent() > MAX_END_CURRENT || end.getBottom().getOutputCurrent() > MAX_END_CURRENT;

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
