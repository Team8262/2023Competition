// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.jumprobotics.arm.TwoJointArm;
import org.jumprobotics.arm.Arm.Method;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax base1, base2, arm;
  SparkMaxPIDController baseController, armController;
  private TwoJointArm armModel;
  /** Creates a new Arm. */
  public Arm() {
    //Arm at natural resting: 19in forward, 9in up
    //Bounding box is roughlty 14in horizontal by 11.5in vertical
    //Starting angles: arm 1 at 90 deg, arm 2 at 30 deg
    //Arm lengths are 36in
    base1 = new CANSparkMax(1, MotorType.kBrushless);
    base2 = new CANSparkMax(23, MotorType.kBrushless);
    arm = new CANSparkMax(21, MotorType.kBrushless);

    base2.follow(base1, false);

    baseController = base1.getPIDController();
    armController = arm.getPIDController();

    baseController.setP(BASE_LINK_VELOCITY_P_CONTROLLER);
    baseController.setI(BASE_LINK_VELOCITY_I_CONTROLLER);
    baseController.setD(BASE_LINK_VELOCITY_D_CONTROLLER);
    baseController.setFF(BASE_LINK_VELOCITY_F_CONTROLLER);
    baseController.setSmartMotionMaxVelocity(BASE_LINK_MAX_VELOCITY, 0);
    baseController.setSmartMotionMaxAccel(BASE_LINK_MAX_ACCELERATION, 0);

    armController.setP(UPPER_LINK_VELOCITY_P_CONTROLLER);
    armController.setI(UPPER_LINK_VELOCITY_I_CONTROLLER);
    armController.setD(UPPER_LINK_VELOCITY_D_CONTROLLER);
    armController.setFF(UPPER_LINK_VELOCITY_F_CONTROLLER);
    armController.setSmartMotionMaxVelocity(UPPER_LINK_MAX_VELOCITY, 0);
    armController.setSmartMotionMaxAccel(UPPER_LINK_MAX_ACCELERATION, 0);

    armModel = new TwoJointArm(BASE_LINK_LENGTH, UPPER_LINK_LENGTH);

  }

  public double[] getCurrentAngles(){
    double[] angles = {base1.getEncoder().getPosition(), arm.getEncoder().getPosition()};
    return angles;
  }

  public void setPosition(Translation2d pos){

    double[][] positions = armModel.toAngles(pos, Method.INVERSE_KINEMATICS);

    double[] pos1 = positions[0];
    double[] pos2 = positions[1];

    double[] currentAngles = getCurrentAngles();

    double dist1 = Math.abs(currentAngles[0] - pos1[0]) + Math.abs(currentAngles[1] - pos1[1]);
    double dist2 = Math.abs(currentAngles[0] - pos2[0]) + Math.abs(currentAngles[1] - pos2[1]);

    double[] posToUse = dist1 < dist2 ? pos1 : pos2;

    double basePos = posToUse[0] * BASE_LINK_GEAR_RATIO / 2*Math.PI;
    double armPos = posToUse[1] * UPPER_LINK_GEAR_RATIO / 2*Math.PI;


    base1.getPIDController().setReference(basePos, ControlType.kSmartMotion);
    arm.getPIDController().setReference(armPos, ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
