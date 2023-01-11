// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
                             DoubleSupplier  translationXSupplier,
                             DoubleSupplier  translationYSupplier,
                             DoubleSupplier  rotationSupplier) {
      this.m_drivetrainSubsystem = drivetrainSubsystem;
      this.m_translationXSupplier = translationXSupplier;
      this.m_translationYSupplier = translationYSupplier;
      this.m_rotationSupplier = rotationSupplier;
      addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
        m_drivetrainSubsystem.drive(m_translationXSupplier.getAsDouble(),
                                    m_translationYSupplier.getAsDouble(),
                                    m_rotationSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(0,0,0);
  }
}
