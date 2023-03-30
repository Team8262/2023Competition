// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import org.jumprobotics.gyro.GyroIO;
import org.jumprobotics.gyro.GyroIONavX;
import org.jumprobotics.swervedrive.SwerveModule;
import org.jumprobotics.swervedrive.SwerveModuleInterface;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  //private Drivetrain m_drivetrain = new Drivetrain();
  //private CameraContainer cams;
  public static class primaryController{
    private static final Joystick j = new Joystick(0);

    public static JoystickButton resetGyroButton(){
      return new JoystickButton(j,6);
    }

    public static JoystickButton xStanceButton(){
      return new JoystickButton(j, 4);
    }

    public static Joystick getJoystick(){
      return j;
    }

    public static  JoystickButton intakeCubeButton(){
      return new JoystickButton(j, 3);
    }

    public static JoystickButton intakeConeButton(){
      return new JoystickButton(j, 2);
    }

    public static JoystickButton spitOutButton(){
      return new JoystickButton(j, 1);
    }

    public static double getSpeedModifier(){
      return 0.5/(j.getRawAxis(2)+1);
    }

    public static JoystickButton testButton(){
      return new JoystickButton(j, 3);
    }

  }

  public static class secondaryController{
    private static final Joystick j = new Joystick(1);

    public static Joystick getJoystick(){
      return j;
    }

    public static JoystickButton scoreHigh(){
      return new JoystickButton(j, 5);
    }

    public static JoystickButton scoreLow() {
      return new JoystickButton(j, 3);
    }

    public static JoystickButton returnHome(){
      return new JoystickButton(j, 2);
    }

    //public static JoystickButton 
    /*
    public static JoystickButton runNewAuto() {
      return new JoystickButton(j, 1);
    }*/



  }

  private Drivetrain m_drivetrain;
  //public static Joystick primaryJoystick = new Joystick(0);
  private Intake intake = new Intake();
  private End end = new End();
  private Arm arm = new Arm();
  //private Vision vision;

  //public Vision getVision() {
  //  return vision;
  //}

  public Drivetrain getDrivetrain(){
    return m_drivetrain;
  }

  //Currently using raw (rotations)
  private HashMap<String, double[][]> armPaths = new HashMap<String, double[][]>();
  
  public RobotContainer() {

    armPaths.put("high", new double[][]{{20,-1},{44,35}});
    armPaths.put("low", new double[][]{{10,-2},{0,0}});
    armPaths.put("home", new double[][]{{10,-2},{0,0}});


    buildRobot();

    // double forward = 0.0;
    DoubleSupplier forwardsupp = () -> primaryController.getSpeedModifier()*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.forwardAxis)) * MAX_VELOCITY_METERS_PER_SECOND * driveSpeedCap;
     
    DoubleSupplier strafesupp = () -> primaryController.getSpeedModifier()*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.strafeAxis)) * MAX_VELOCITY_METERS_PER_SECOND * driveSpeedCap;
    
    DoubleSupplier rotatesupp = () -> 0.5*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.rotationAxis)) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * rotationSpeedCap;
    
    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrain, 
            forwardsupp,
            strafesupp,
            rotatesupp
    ));

    DoubleSupplier baseSupp = () -> modifyAxis(getSecondaryJoystick().getRawAxis(0))*MAX_ANGULAR_SPEED;
    DoubleSupplier armSupp = () -> modifyAxis(getSecondaryJoystick().getRawAxis(1))*MAX_ANGULAR_SPEED;

    //arm.setDefaultCommand(new ManualArmControl(arm, baseSupp, armSupp));
    /* 
    try {
      vision = new Vision();
    } catch(IOException e) {
      System.err.println("vision shatted itself");
    }*/



    configureBindings();
  }

  public Joystick getPrimaryJoystick(){
    return primaryController.getJoystick();
  }

  public Joystick getSecondaryJoystick(){
    return secondaryController.getJoystick();
  }

  private void configureBindings() {

    primaryController.resetGyroButton().onTrue(Commands.runOnce(m_drivetrain::zeroGyroscope, m_drivetrain));
    primaryController.xStanceButton().onTrue(Commands.runOnce(m_drivetrain::enableXstance, m_drivetrain));
    primaryController.xStanceButton().onFalse(Commands.runOnce(m_drivetrain::disableXstance, m_drivetrain));
  
   
    primaryController.intakeConeButton().whileTrue(new InstantCommand(() -> coneIntake(1)));
    primaryController.intakeConeButton().whileFalse(new InstantCommand(() -> coneIntake(0.0)));
    primaryController.intakeCubeButton().whileTrue(new InstantCommand(() -> cubeIntake(1)));
    primaryController.intakeCubeButton().whileFalse(new InstantCommand(() -> cubeIntake(0.0)));
   
    primaryController.spitOutButton().whileTrue(new InstantCommand(() -> spitout(1)));
    primaryController.spitOutButton().whileFalse(new InstantCommand(() -> spitout(0.0)));

    //primaryController.testButton().whileTrue(new AutoBalance(m_drivetrain));

    secondaryController.scoreHigh().whileTrue(new FollowArmPath(arm, armPaths.get("high")));
    secondaryController.returnHome().whileTrue(new FollowArmPath(arm, armPaths.get("home")));
    secondaryController.scoreLow().whileTrue(new FollowArmPath(arm, armPaths.get("low")));
    //secondaryController.runNewAuto().onTrue(new GoToAuto(1, m_drivetrain, m_drivetrain.getPose()));

  }

  

  public void coneIntake(double speed){
    intake.setSpeed(speed);
    //end.setConeSpeed(speed);
  }

  public void cubeIntake(double speed){
    intake.setSpeed(speed);
    end.setCubeSpeed(speed);
  }

  public void spitout(double speed){
    intake.setSpeed(-speed);
    end.setCubeSpeed(-speed);
    //end.setConeSpeed(-speed);
  }

  
  

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    //Auto2 auto = new Auto2(m_drivetrain, this);
    return new InstantCommand();//auto;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private void buildRobot(){
    GyroIO gyro = new GyroIONavX();
    SwerveModule flModule =
    new SwerveModule(
        new SwerveModuleInterface(
            0,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET),
        0,
        MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule frModule =
        new SwerveModule(
            new SwerveModuleInterface(
                1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET),
            1,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule blModule =
        new SwerveModule(
            new SwerveModuleInterface(
                2,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET),
            2,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule brModule =
        new SwerveModule(
            new SwerveModuleInterface(
                3,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET),
            3,
            MAX_VELOCITY_METERS_PER_SECOND);

    m_drivetrain = new Drivetrain(this, gyro, flModule, frModule, blModule, brModule);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.02);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
