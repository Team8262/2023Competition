// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;
  private boolean robotOriented = true;
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
  public void execute() {        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
      robotOriented = !(SmartDashboard.getBoolean("Field Oriented", true));
      
      robotOriented = false;
      if(robotOriented){
          m_drivetrainSubsystem.drive(
                  new ChassisSpeeds(m_translationXSupplier.getAsDouble(),
                                      m_translationYSupplier.getAsDouble(),
                                      m_rotationSupplier.getAsDouble())
          );  
      }else{
          m_drivetrainSubsystem.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                          m_translationXSupplier.getAsDouble(),
                          m_translationYSupplier.getAsDouble(),
                          m_rotationSupplier.getAsDouble(),
                          m_drivetrainSubsystem.getGyroscopeRotation()
                  )
          );  
      }
      SmartDashboard.putNumber("Gyro Angle ", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
      SmartDashboard.putNumber("Drive Strafe Input", m_translationXSupplier.getAsDouble());
      SmartDashboard.putNumber("Drive Forward Input", m_translationYSupplier.getAsDouble());
      SmartDashboard.putNumber("Drive Rotation Input", m_rotationSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
