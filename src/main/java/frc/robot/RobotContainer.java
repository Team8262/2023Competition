// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import org.jumprobotics.gyro.GyroIO;
import org.jumprobotics.gyro.GyroIONavX;
import org.jumprobotics.swervedrive.SwerveModule;
import org.jumprobotics.swervedrive.SwerveModuleInterface;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static class primaryController{
    private static final Joystick j = new Joystick(0);

    public static JoystickButton resetGyroButton(){
      return new JoystickButton(j,1);
    }

    public static JoystickButton xStanceButton(){
      return new JoystickButton(j, 2);
    }

    public static JoystickButton intakeButton(){
      return new JoystickButton(j, 4);
    }

    public static double getForwardAxis(){
      return j.getRawAxis(1);
    }

    public static double getStrafeAxis(){
      return j.getRawAxis(0);
    }

    public static double getRotationAxis(){
      return j.getRawAxis(2);
    }

    public static Joystick getJoystick(){
      return j;
    }
  }

  public static class secondaryController{
    private static final Joystick j = new Joystick(1);

    public static double getFirstAxis(){
      return j.getRawAxis(0);
    }

    public static double getSecondAxis(){
      return j.getRawAxis(1);
    }

    public static Joystick geJoystick(){
      return j;
    }
  }

  private Drivetrain m_drivetrain;
  private Arm m_arm;
  private Intake intake;
  private End end;

  public RobotContainer() {

        buildRobot();

        // double forward = 0.0;
        DoubleSupplier forwardsupp = () -> 1*modifyAxis(primaryController.getForwardAxis()) * MAX_VELOCITY_METERS_PER_SECOND * driveSpeedCap;
     
        DoubleSupplier strafesupp = () -> 1*modifyAxis(primaryController.getStrafeAxis()) * MAX_VELOCITY_METERS_PER_SECOND * driveSpeedCap;
    
        DoubleSupplier rotatesupp = () -> 1*modifyAxis(primaryController.getRotationAxis()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * rotationSpeedCap;
    
        m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrain, 
                forwardsupp,
                strafesupp,
                rotatesupp
        ));


        DoubleSupplier firstSupp = () -> 1*modifyAxis(secondaryController.getFirstAxis());
        DoubleSupplier secondSupp = () -> 1*modifyAxis(secondaryController.getSecondAxis());

        m_arm.setDefaultCommand(new TestArmCommand(m_arm, firstSupp, secondSupp));
        

    configureBindings();
  }

  private void configureBindings() {

    primaryController.resetGyroButton().onTrue(Commands.runOnce(m_drivetrain::zeroGyroscope, m_drivetrain));
    primaryController.xStanceButton().onTrue(Commands.runOnce(m_drivetrain::enableXstance, m_drivetrain));
    primaryController.xStanceButton().onFalse(Commands.runOnce(m_drivetrain::disableXstance, m_drivetrain));

    primaryController.intakeButton().whileTrue(new IntakeThings(intake, end));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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

    m_arm = new Arm();

    intake = new Intake();

    end = new End();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.025);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
