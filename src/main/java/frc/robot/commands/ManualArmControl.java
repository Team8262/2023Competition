// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

public class ManualArmControl extends CommandBase {

  private final Arm m_Arm;
  private final DoubleSupplier m_xSupp;
  private final DoubleSupplier m_ySupp;
  private double upperTarget, lowerTarget;

  /** Creates a new DefaultArmCommand. */
  public ManualArmControl(Arm armSubsystem, DoubleSupplier xSupp, DoubleSupplier ySupp) {
    this.m_Arm = armSubsystem;
    this.m_xSupp = xSupp;
    this.m_ySupp = ySupp;
    upperTarget = 0;
    lowerTarget=0;
    
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double[] positions = m_Arm.getCurrentPositions();
    lowerTarget+=m_xSupp.getAsDouble()*MAX_ANGULAR_SPEED;
    upperTarget+=m_ySupp.getAsDouble()*MAX_ANGULAR_SPEED;
    if(lowerTarget<=0){
      lowerTarget=0;
    }
    if(upperTarget<=0){
      upperTarget=0;
    }

    //m_Arm.setAngles(lowerTarget, upperTarget);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}