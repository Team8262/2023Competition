// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;


public class End extends SubsystemBase {

  private CANSparkMax bottomMotor, topMotor;

  /** Creates a new End. */
  public End() {
    bottomMotor = new CANSparkMax(END_BOTTOM, MotorType.kBrushless);
    topMotor = new CANSparkMax(END_TOP, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    bottomMotor.set(speed);
    topMotor.set(-1*speed);
  }
}
