// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.jumprobotics.arm.TwoJointArm;
import org.jumprobotics.arm.Arm.Method;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public CANSparkMax base1, base2, arm;
  private SparkMaxPIDController baseController, armController;
  private RelativeEncoder baseEncoder, armEncoder;
  private TwoJointArm armModel;

  private static final String SUBSYSTEM_NAME = "Arm";
  
  private static final boolean DEBUGGING = true;

  //angles
  private double base_angle;
  private double upper_angle;

   
  /** Creates a new Arm. */
  public Arm() {

    base1 = new CANSparkMax(1, MotorType.kBrushless);
    base2 = new CANSparkMax(23, MotorType.kBrushless);
    arm = new CANSparkMax(21, MotorType.kBrushless);

    base_angle = 0.0;
    upper_angle = 0.0;

    base2.follow(base1, false);

    baseController = base1.getPIDController();
    armController = arm.getPIDController();

    //TODO set real limits---these are arbitrary
    base1.setSoftLimit(SoftLimitDirection.kForward, 2);
    base1.setSoftLimit(SoftLimitDirection.kReverse, 2);
    arm.setSoftLimit(SoftLimitDirection.kForward, 2);
    arm.setSoftLimit(SoftLimitDirection.kReverse, 2);

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

    //baseEncoder = base1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    //armEncoder = arm.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    baseEncoder = base1.getEncoder();
    armEncoder = arm.getEncoder();
    baseEncoder.setPosition(2); //These are fake numbers... replace them with init values in rotations
    armEncoder.setPosition(2);

    armModel = new TwoJointArm(BASE_LINK_LENGTH, UPPER_LINK_LENGTH);
    armModel.addLookupTable("ArmLookupTable.json");

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.addNumber("Base Degrees", () -> baseEncoder.getPosition()*360);
      tab.addNumber("Arm Degrees", () -> armEncoder.getPosition()*360);
    }
  }

  public TwoJointArm getArmModel(){
    return armModel;
  }

  /*
   * return the current position as radians
   */
  public double[] getCurrentPositions(){
    double[] angles = {baseEncoder.getPosition()*2*Math.PI/BASE_LINK_GEAR_RATIO, armEncoder.getPosition() * 2 * Math.PI/UPPER_LINK_GEAR_RATIO};
    return angles;
  }

  public void setPosition(Translation2d pos){

    double[][] positions = armModel.toAngles(pos, Method.INVERSE_KINEMATICS);

    double[] pos1 = positions[0];
    double[] pos2 = positions[1];

    double[] currentAngles = getCurrentPositions();

    double dist1 = Math.abs(currentAngles[0] - pos1[0]) + Math.abs(currentAngles[1] - pos1[1]);
    double dist2 = Math.abs(currentAngles[0] - pos2[0]) + Math.abs(currentAngles[1] - pos2[1]);

    double[] posToUse = dist1 < dist2 ? pos1 : pos2;

    double basePos = posToUse[0] * BASE_LINK_GEAR_RATIO / (2*Math.PI);
    double armPos = posToUse[1] * UPPER_LINK_GEAR_RATIO / (2*Math.PI);
    
    arm.getPIDController().setReference(armPos, ControlType.kSmartMotion,0,getUpperFF());
    base1.getPIDController().setReference(basePos, ControlType.kSmartMotion,0,getBaseFF());
  }

  //In radians
  public void setAngles(double lower, double upper){
    // order matters here
    armController.setReference(upper*UPPER_LINK_GEAR_RATIO/(2*Math.PI), ControlType.kSmartMotion,0,getUpperFF());
    baseController.setReference(lower*BASE_LINK_GEAR_RATIO/(2*Math.PI), ControlType.kSmartMotion,0,getBaseFF());
  }

  public void moveTo(double lower, double upper, double speed) {
    //
  }

  //second
  private double getBaseFF(){
    base_angle = baseEncoder.getPosition()*BASE_LINK_GEAR_RATIO;
    double torque = (Math.cos(base_angle*2*Math.PI)*BASE_CENTER_OF_MASS*BASE_MASS+(Math.cos(upper_angle*2*Math.PI)*UPPER_CENTER_OF_MASS+Math.cos(base_angle*2*Math.PI)*BASE_LENGTH)*UPPER_MASS);
    
    //TODO movement feedforward 

    return torque*BASE_VOLTAGE_COMPENSATION;
  }

  //first
  private double getUpperFF(){
    upper_angle = armEncoder.getPosition()*UPPER_LINK_GEAR_RATIO;
    double torque = Math.cos(upper_angle*2*Math.PI)*UPPER_CENTER_OF_MASS*UPPER_MASS;
    //TODO movement feedforward 
    return torque*UPPER_VOLTAGE_COMPENSATION;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
