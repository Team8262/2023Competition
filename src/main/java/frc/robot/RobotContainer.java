// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
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
import frc.robot.commands.Auto1;

public class RobotContainer {

  //private Drivetrain m_drivetrain = new Drivetrain();
  private CameraContainer cams;
  public static class primaryController{
    private static final Joystick j = new Joystick(0);

    public static JoystickButton resetGyroButton(){
      return new JoystickButton(j,1);
    }

    public static JoystickButton xStanceButton(){
      return new JoystickButton(j, 2);
    }

    public static Joystick getJoystick(){
      return j;
    }

    private static  JoystickButton intakeCubeButton(){
      return new JoystickButton(j, 6);
    }

    private static JoystickButton intakeConeButton(){
      return new JoystickButton(j, 5);
    }

  }

  private Drivetrain m_drivetrain;
  public static Joystick primaryJoystick = new Joystick(0);
  private Intake intake;
  private End end;

  public RobotContainer() {

        buildRobot();

        // double forward = 0.0;
        DoubleSupplier forwardsupp = () -> 1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.forwardAxis)) * MAX_VELOCITY_METERS_PER_SECOND * driveSpeedCap;
     
        DoubleSupplier strafesupp = () -> 1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.strafeAxis)) * MAX_VELOCITY_METERS_PER_SECOND * driveSpeedCap;
    
         DoubleSupplier rotatesupp = () -> 1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.rotationAxis)) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * rotationSpeedCap;
    
        m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrain, 
                forwardsupp,
                strafesupp,
                rotatesupp
        ));

        try {
          cams = new CameraContainer();
        } catch(IOException e) {
          
        }



    configureBindings();
  }

  public Joystick getPrimaryJoystick(){
    return primaryJoystick;
  }

  private void configureBindings() {

    primaryController.resetGyroButton().onTrue(Commands.runOnce(m_drivetrain::zeroGyroscope, m_drivetrain));
    primaryController.xStanceButton().onTrue(Commands.runOnce(m_drivetrain::enableXstance, m_drivetrain));
    primaryController.xStanceButton().onFalse(Commands.runOnce(m_drivetrain::disableXstance, m_drivetrain));

    primaryController.intakeConeButton().whileTrue(new InstantCommand(() -> coneIntake(1.0)));
    primaryController.intakeConeButton().whileFalse(new InstantCommand(() -> coneIntake(0.0)));
    primaryController.intakeCubeButton().whileTrue(new InstantCommand(() -> cubeIntake(1.0)));
    primaryController.intakeCubeButton().whileFalse(new InstantCommand(() -> cubeIntake(0.0)));
    

  }

  public void coneIntake(double speed){
    intake.setSpeed(speed);
    end.setConeSpeed(speed);
  }

  public void cubeIntake(double speed){
    intake.setSpeed(speed);
    end.setCubeSpeed(speed);
  }


  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    // Auto1 auto = new Auto1(m_drivetrain);
    return new Auto1(m_drivetrain);
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

    m_drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.025);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
