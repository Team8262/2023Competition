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
import org.jumprobotics.gyro.GyroIOPigeon;
import org.jumprobotics.swervedrive.SwerveModule;
import org.jumprobotics.swervedrive.SwerveModuleInterface;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    public static  JoystickButton intakeButton(){
      return new JoystickButton(j, 3);
    }

    public static JoystickButton spitOutButton(){
      return new JoystickButton(j, 1);
    }

    public static double getSpeedModifier(){
      return 1/(j.getRawAxis(2)+1);
    }

    public static JoystickButton testButton(){
      return new JoystickButton(j, 2);
    }

    private static JoystickButton switchMap() {
      return new JoystickButton((j), 5);
    }

  }

  public static class secondaryController{
    private static final Joystick j = new Joystick(1);

    public static Joystick getJoystick(){
      return j;
    }

    public static JoystickButton scoreHigh(){
      return new JoystickButton(j, 3);
    }

    public static JoystickButton returnHome(){
      return new JoystickButton(j,4);
    }

    public static JoystickButton scoreMid(){
      return new JoystickButton(j,5);
    }

    public static JoystickButton coneTrayando(){
      return new JoystickButton(j, 6);
    }
    public static JoystickButton zerooo(){
      return new JoystickButton(j, 10);
    }

    public static JoystickButton intake(){
      return new JoystickButton(j, 1);
    }

    public static JoystickButton spitout(){
      return new JoystickButton(j, 2);
    }

    //public static JoystickButton 
    /*
    public static JoystickButton runNewAuto() {
      return new JoystickButton(j, 1);
    }*/



  }

  private static Drivetrain m_drivetrain;

  //public static Joystick primaryJoystick = new Joystick(0);
  private Intake intake = new Intake();
  public End end = new End();
  public Arm arm = new Arm();
  //private Vision vision;

  public static Drivetrain getDrivetrain(){
    return m_drivetrain;
  }

  //Currently using raw (rotations)
  public static HashMap<Integer, double[][]> armPaths = new HashMap<Integer, double[][]>();
  
  public RobotContainer() {
    //cones are stowed dfferentl than cubes
    //for cubes the arm is stuck in the intake so we need 3 points
    //for cones the arm should be able to mvoe directly to the good position
    //im using variable because the monkey who wrote this ebfore me
    //was just spamming unumbers
    //wesley you are being carried by new software
    double[] home = new double[]{0,0}; // stow position for the cubbbbes
    double[] homeMid = new double[]{-.2,0};//,-1}; //midway out of the stow position
    double[] homeMid2 = new double[]{-.15, 0};
    //double[] homeOut = homeMid; //new double[]{-5,-5}; // out of the stowed position free to move

    double[] coneHome = new double[]{0.05,-.91}; //stow position for the arm in cone mode you fucking idiot
    //armPaths.put(1, new double[][]{homeMid, {-.226, -.581}}); // high cube
    armPaths.put(1, new double[][]{homeMid, {-.200, -.581}}); // high cube
    //armPaths.put("high", new double[][]{homeMid, {-.4, 0}});
    armPaths.put(3, new double[][]{homeMid, {-.156,-.72}}); // mid cube
    armPaths.put(2, new double[][]{{-.137,-.709}}); // high cone
    armPaths.put(6, new double [][]{{-.2105,-.568}});// cone mid
    armPaths.put(7, new double[][]{homeMid, {-0.1687487,-.619605}}); //trsyyy
    armPaths.put(14, new double[][]{homeMid, {-0.1687487,-.619605}}); //trsyyy alternate
    //armPaths.put(5, new double[][]{{-.3, 0}, homeMid,homeMid2, {-.1, 0}, {-.05, 0},home}); // homee
    armPaths.put(5, new double[][]{{-0.25,0},{-0.2,0}, {-0.1,0}, {-0.1,0},{-0.05,0},{0,0.02}}); //Temporary for testing
    armPaths.put(10, new double[][]{coneHome}); // conehomee
    armPaths.put(11, new double[][]{{-.5,-.5}});

    armPaths.put(9, new double[][]{home}); // direct home

    buildRobot();

    // double forward = 0.0;
    DoubleSupplier forwardsupp = () -> primaryController.getSpeedModifier()*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.forwardAxis)) * MAX_VELOCITY_METERS_PER_SECOND;
     
    DoubleSupplier strafesupp = () -> primaryController.getSpeedModifier()*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.strafeAxis)) * MAX_VELOCITY_METERS_PER_SECOND;
    
    DoubleSupplier rotatesupp = () -> 0.35*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.rotationAxis)) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;


    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrain, 
            forwardsupp,
            strafesupp,
            rotatesupp    ));

    DoubleSupplier baseSupp = () -> modifyAxis(getSecondaryJoystick().getRawAxis(0))*MAX_ANGULAR_SPEED;
    DoubleSupplier armSupp = () -> modifyAxis(getSecondaryJoystick().getRawAxis(1))*MAX_ANGULAR_SPEED;

    //arm.setDefaultCommand(new ManualArmControl(arm, baseSupp, armSupp));
     


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
  
   
    //primaryController.intakeConeButton().whileTrue(new InstantCommand(() -> coneIntake(1)));
    //primaryController.intakeConeButton().whileFalse(new InstantCommand(() -> coneIntake(0.0)));
    primaryController.intakeButton().whileTrue(new InstantCommand(() -> intake(-1)));
    primaryController.intakeButton().whileFalse(new InstantCommand(() -> intake(0.0)));
   
    primaryController.spitOutButton().whileTrue(new InstantCommand(() -> spitout(-1)));
    primaryController.spitOutButton().whileFalse(new InstantCommand(() -> spitout(0.0)));

    //primaryController.switchMap().whileTrue(new InstantCommand(() -> setDrivingMode(1)));
    //primaryController.switchMap().whileFalse(new InstantCommand(() -> setDrivingMode(0)));


    primaryController.testButton().whileTrue(new AutoBalance(m_drivetrain));
    
    
    secondaryController.scoreHigh().whileTrue(new gayArmPath(arm, 1, 2.5));
    secondaryController.returnHome().whileTrue(new gayArmPath(arm, 5, 1));
    secondaryController.scoreMid().whileTrue(new gayArmPath(arm, 3, 2.5));

    //Cones
    //secondaryController.scoreHigh().whileTrue(new gayArmPath(arm, 2, 2.5));
    //secondaryController.returnHome().whileTrue(new gayArmPath(arm, 10, 1));
    //secondaryController.scoreMid().whileTrue(new gayArmPath(arm, 6, 2.5));
    

    secondaryController.coneTrayando().whileTrue(new gayArmPath(arm, 7, 1));
    secondaryController.spitout().whileTrue(new InstantCommand(() -> end.setSpeed(-1)));
    secondaryController.spitout().whileFalse(new InstantCommand(() -> end.setSpeed(0.0)));
    secondaryController.intake().whileTrue(new InstantCommand(() -> end.setSpeed(0.4)));
    secondaryController.intake().whileFalse(new InstantCommand(() -> end.setSpeed(0)));


    
    //secondaryController.runNewAuto().onTrue(new GoToAuto(1, m_drivetrain, m_drivetrain.getPose()));

  }

  public void setDrivingMode(int i) {
    //m_drivetrain.drivingMode = i;
  }
  public void intake(double speed){
    intake.setSpeed(speed);
    end.setSpeed(-speed);
  }

  public void spitout(double speed){
    intake.setSpeed(-speed);
    end.setSpeed(speed);
    //end.setConeSpeed(-speed);
  }

  public static boolean cancel() {
    Joystick stik = new Joystick(1);
    return stik.getRawButton(12);
  }

  public Command onylArmAuto(){

    new InstantCommand(() -> new gayArmPath(arm, 1, 1)); 
    new WaitCommand(1.5);
    new InstantCommand(() -> spitout(1));
    new WaitCommand(1.5);
    new InstantCommand(() -> new gayArmPath(arm, 5, 0.5));
    new InstantCommand(() -> spitout(0));

    return new PrintCommand("works");
  }
  

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");

    // return onylArmAuto();
    // return new Auto_mov_roobr(m_drivetrain, this);
    return new BadAuto(this);
    
    //return new gayAuto(m_drivetrain, this);
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
    //GyroIO gyro = new GyroIONavX();
    GyroIO gyro = new GyroIOPigeon(17);
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
