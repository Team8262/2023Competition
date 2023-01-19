// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DefaultArmCommand extends CommandBase {

  private final Arm m_Arm;
  private final DoubleSupplier m_xSupp;
  private final DoubleSupplier m_ySupp;
  private Translation2d position;

  /** Creates a new DefaultArmCommand. */
  public DefaultArmCommand(Arm armSubsystem, DoubleSupplier xSupp, DoubleSupplier ySupp) {
    this.m_Arm = armSubsystem;
    this.m_xSupp = xSupp;
    this.m_ySupp = ySupp;
    
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = new Translation2d(position.getX() + m_xSupp.getAsDouble(), position.getY() + m_ySupp.getAsDouble());
    
    m_Arm.setPosition(position);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setPosition(new Translation2d(0.48,0.48));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
