// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightVision;

public class DefaultDriveCommand extends CommandBase {

  private final Drivetrain m_drivetrainSubsystem;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;


  private final LimelightVision vision;
  private final PIDController forwardPidController;
  private final PIDController horizontalPidController;
  private final PIDController rotationPidController;
  private double ReflectiveTapeYposition = 2;
  private double ReflectiveTapeXposition = 0;
  private double HumanPlayerYpositiion = 2;
  private double HumanPlayerXposiiton = 0;
  private double yError = 0;
  private double xError = 0;
  private int drivingMode = 0;
  
  public boolean DEBUGGING = true;

  public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
                             DoubleSupplier  translationXSupplier,
                             DoubleSupplier  translationYSupplier,
                             DoubleSupplier  rotationSupplier, 
                             LimelightVision vision) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.vision = vision;
    addRequirements(drivetrainSubsystem);

    forwardPidController = new PIDController(0, 0, 0);
    horizontalPidController = new PIDController(/*-3*/1, 0, 0);
    rotationPidController = new PIDController(/*-0.5*/0, 0, 0);
        
    if(DEBUGGING) {
      ShuffleboardTab robotAlignment = Shuffleboard.getTab("Align_Robot");
      robotAlignment.addDouble("forward PID Position Error: ", () -> forwardPidController.getPositionError());
      robotAlignment.addDouble("horizontal PID Position Error: ", () -> horizontalPidController.getPositionError());
      robotAlignment.addDouble("rotational PID Position Error: ", () -> rotationPidController.getPositionError());
      robotAlignment.addDouble("yError: ", () -> yError);
      robotAlignment.addDouble("xError: ", () -> xError); 
    }
  }

  @Override
  public void execute() {

    yError = (Constants.POLE_HEIGHT - Constants.CAMERA_HEIGHT) / Math.tan(Math.toRadians(20 + vision.getTargetY()));
    xError = yError * Math.cos(Math.toRadians(vision.getTargetX())+10);
    System.out.println("xerror: "+ xError);

    if(m_drivetrainSubsystem.drivingMode == 0) {
      m_drivetrainSubsystem.drive(m_translationXSupplier.getAsDouble(),
                                  m_translationYSupplier.getAsDouble(),
                                  m_rotationSupplier.getAsDouble());
    } if(m_drivetrainSubsystem.drivingMode == 1) {  
      System.out.println("hiiiii");
      yError = (Constants.POLE_HEIGHT - Constants.CAMERA_HEIGHT) / Math.tan(Math.toRadians(20 + vision.getTargetY()));
      xError = yError * Math.cos(Math.toRadians(vision.getTargetX())+10);

      m_drivetrainSubsystem.drive( //forward is disabled to test horizontal movement
            forwardPidController.calculate(yError, ReflectiveTapeXposition),
            horizontalPidController.calculate(xError, 0),
            rotationPidController.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), 0));
    } if(m_drivetrainSubsystem.drivingMode == 2) {

      m_drivetrainSubsystem.drive(
        forwardPidController.calculate(yError, HumanPlayerYpositiion),
        horizontalPidController.calculate(xError, HumanPlayerXposiiton),
        rotationPidController.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), 0)
      );

    }
  }

  @Override
  public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(0,0,0);
  }

  /*
   * -1: nothing
   * 0: driving
   * 1: aligining with reflective tape
   * 2: aligning with human player station
   */

  public void setMode(int i) {
    System.out.println("hjafdjkhruihauhgauohdsufdas");
    drivingMode = i;
    if(i == 1) { //sets limelight to reflective tape pipeline
      vision.setPipelineIndex(0);
    }
  }
}
