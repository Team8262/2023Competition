// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TestArmCommand extends CommandBase {

  private final Arm m_Arm;
  private final DoubleSupplier m_xSupp;
  private final DoubleSupplier m_ySupp;

  /** Creates a new TestArmCommand. */
  public TestArmCommand(Arm armSubsystem, DoubleSupplier xSupp, DoubleSupplier ySupp) {
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
    m_Arm.base1.set(m_xSupp.getAsDouble());
    m_Arm.arm.set(m_ySupp.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.base1.set(0);
    m_Arm.arm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
