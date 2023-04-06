// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

import java.nio.DoubleBuffer;
import java.lang.Math;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.AlternateEncoderType;
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
  public CANSparkMax base1, base2, arm, arm2;
  private SparkMaxPIDController baseController, armController;
  private RelativeEncoder baseEncoder, armEncoder;

  private static final String SUBSYSTEM_NAME = "Arm";
  
  private static final boolean DEBUGGING = true;

  //angles
  private double base_angle;
  private double upper_angle;

   
  /** Creates a new Arm. */
  public Arm() {

    base1 = new CANSparkMax(33, MotorType.kBrushless);
    base2 = new CANSparkMax(23, MotorType.kBrushless);
    arm = new CANSparkMax(21, MotorType.kBrushless);
    arm2 = new CANSparkMax(29, MotorType.kBrushless);

    //ANTI TOMMY SYSTEM

    int stallLimit = 6;
    int freeLimit = 10;

    arm.setSmartCurrentLimit(stallLimit, freeLimit);
    arm2.setSmartCurrentLimit(stallLimit, freeLimit);
    arm.setIdleMode(IdleMode.kBrake);
    arm2.setIdleMode(IdleMode.kBrake);
    base1.setSmartCurrentLimit(stallLimit, freeLimit);
    base2.setSmartCurrentLimit(stallLimit, freeLimit);
    base1.setIdleMode(IdleMode.kBrake);
    base2.setIdleMode(IdleMode.kBrake);
    base_angle = 0.0;
    upper_angle = 0.0;

    base2.setInverted(false);

    arm2.follow(arm, true); 

    base1.follow(base2, true);

    baseController = base2.getPIDController();
    armController = arm.getPIDController();

    //TODO set real limits---these are arbitrary
    base2.setSoftLimit(SoftLimitDirection.kForward, (float) (2*BASE_LINK_GEAR_RATIO));
    base2.setSoftLimit(SoftLimitDirection.kReverse, (float) (2*BASE_LINK_GEAR_RATIO));
    arm.setSoftLimit(SoftLimitDirection.kForward, (float) (2*UPPER_LINK_GEAR_RATIO));
    arm.setSoftLimit(SoftLimitDirection.kReverse, (float) (2*UPPER_LINK_GEAR_RATIO));

    arm.enableSoftLimit(SoftLimitDirection.kForward, false);
    arm.enableSoftLimit(SoftLimitDirection.kReverse, false);
    base2.enableSoftLimit(SoftLimitDirection.kForward, false);
    base2.enableSoftLimit(SoftLimitDirection.kReverse, false);

    baseController.setP(BASE_LINK_VELOCITY_P_CONTROLLER);
    baseController.setI(BASE_LINK_VELOCITY_I_CONTROLLER);
    baseController.setD(BASE_LINK_VELOCITY_D_CONTROLLER);
    baseController.setFF(BASE_LINK_VELOCITY_F_CONTROLLER);
    baseController.setSmartMotionMaxVelocity(BASE_LINK_MAX_VELOCITY, 0);
    baseController.setSmartMotionMaxAccel(BASE_LINK_MAX_ACCELERATION, 0);

    baseController.setOutputRange(-0.5, 0.5);

    armController.setP(UPPER_LINK_VELOCITY_P_CONTROLLER);
    armController.setI(UPPER_LINK_VELOCITY_I_CONTROLLER);
    armController.setD(UPPER_LINK_VELOCITY_D_CONTROLLER);
    armController.setFF(UPPER_LINK_VELOCITY_F_CONTROLLER);
    armController.setSmartMotionMaxVelocity(UPPER_LINK_MAX_VELOCITY, 0);
    armController.setSmartMotionMaxAccel(UPPER_LINK_MAX_ACCELERATION, 0);

    armController.setOutputRange(-0.5, 0.5);

    //baseEncoder = base1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    //armEncoder = arm.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    baseEncoder = base2.getEncoder();
    armEncoder = arm.getEncoder();
    baseEncoder.setPosition(0); //These are fake numbers... replace them with init values in rotations
    armEncoder.setPosition(0);

    //armModel.addLookupTable("ArmLookupTable.json");

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.addNumber("Base Raw", () -> baseEncoder.getPosition());
      tab.addNumber("Arm Raw", () -> armEncoder.getPosition());
      tab.addNumber("Base Angle", () -> baseEncoder.getPosition() * 2 * Math.PI / BASE_LINK_GEAR_RATIO);
      tab.addNumber("Arm Angle", () -> armEncoder.getPosition() * 2 * Math.PI / UPPER_LINK_GEAR_RATIO);
    }
  
    base2.set(0);
    arm.set(0);


  }


  /*
   * return the current position as radians
   */
  public double[] getCurrentPositions(){
    double[] angles = {baseEncoder.getPosition()*2*Math.PI/BASE_LINK_GEAR_RATIO, armEncoder.getPosition() * 2 * Math.PI/UPPER_LINK_GEAR_RATIO};
    return angles;
  }

  //return raw in rotations
  public double[] getRawPositions(){
    double[] angles = {baseEncoder.getPosition(), armEncoder.getPosition()};
    return angles;
  }

  /*
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
  }*/
  public double[] getRealAngle(){
    double baseAngle = baseEncoder.getPosition()/BASE_LINK_GEAR_RATIO;
    //Accounts for fourbar by adding the first stage rotation so as to couneract the fourbar effect.
    double[] realAngles = {baseAngle, armEncoder.getPosition()/UPPER_LINK_GEAR_RATIO - baseAngle};
    return realAngles;
  }

  //In radians
  public void setAngles(double lower, double upper){
    // order matters here
    armController.setReference(upper*UPPER_LINK_GEAR_RATIO/(2*Math.PI), ControlType.kPosition,0,getUpperFF());
    baseController.setReference(lower*BASE_LINK_GEAR_RATIO/(2*Math.PI), ControlType.kPosition,0,getBaseFF());
  }

  //In rotations
  public void setsAnglesRaw(double lower, double upper){
    armController.setReference(upper, ControlType.kPosition,0,getUpperFF());
    baseController.setReference(lower, ControlType.kPosition,0,getBaseFF());
    //System.out.println("Goal: " + lower + "    , Actual: " + baseEncoder.getPosition());
  }

  public void gayAnglesRaw(double lower, double upper) {
    double lowerGoal = BASE_LINK_GEAR_RATIO * (lower - getRealAngle()[0]);
    double upperGoal = UPPER_LINK_GEAR_RATIO * (upper - getRealAngle()[1]);
    base2.set(lowerGoal);
    base1.set(lowerGoal);
    arm.set(upperGoal);
    arm2.set(upperGoal);
  }

  public void diee() {
    base2.set(0);
    arm.set(0);
  }

  public void spin() {
    arm.set(-.1);
    arm2.set(-.1);
  }

  //Super basic, probably wsrong
  /* 
  public boolean stowed(){
    return armModel.toPosition(getCurrentPositions()).getY() <= 0.5;
  }*/


  //second
  private double getBaseFF(){
    double angle = baseEncoder.getPosition()*BASE_LINK_GEAR_RATIO;
    //double torque = (Math.cos(base_angle*2*Math.PI)*BASE_CENTER_OF_MASS*BASE_MASS+(Math.cos(upper_angle*2*Math.PI)*UPPER_CENTER_OF_MASS+Math.cos(base_angle*2*Math.PI)*BASE_LENGTH)*UPPER_MASS);
    
    //TODO movement feedforward 

    return Math.sin(angle)*6; //torque*BASE_VOLTAGE_COMPENSATION;
  }

  //first
  private double getUpperFF(){
    double angle = armEncoder.getPosition()*UPPER_LINK_GEAR_RATIO;
    //double torque = Math.cos(upper_angle*2*Math.PI)*UPPER_CENTER_OF_MASS*UPPER_MASS;
    //TODO movement feedforward 
    return Math.sin(angle)*4;//torque*UPPER_VOLTAGE_COMPENSATION;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  //System.out.println(base1.getEncoder().getPosition());
  }
}
