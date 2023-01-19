// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax base1, base2, arm;
  /** Creates a new Arm. */
  public Arm() {
    //Arm at natural resting: 19in forward, 9in up
    //Bounding box is roughlty 14in horizontal by 11.5in vertical
    //Starting angles: arm 1 at 90 deg, arm 2 at 30 deg
    //Arm lengths are 36in
    base1 = new CANSparkMax(1, MotorType.kBrushless);
    base2 = new CANSparkMax(23, MotorType.kBrushless);
    arm = new CANSparkMax(21, MotorType.kBrushless);

  }

  public void setPosition(Translation2d pos){
    double basePos = 0;
    double armPos = 0;

    base1.getPIDController().setReference(basePos, ControlType.kPosition);
    base2.getPIDController().setReference(basePos, ControlType.kPosition);
    arm.getPIDController().setReference(armPos, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
