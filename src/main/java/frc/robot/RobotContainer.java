// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private Drivetrain m_drivetrain = new Drivetrain();
  public static Joystick primaryJoystick = new Joystick(0);

  public RobotContainer() {

        // double forward = 0.0;
        DoubleSupplier forwardsupp = () -> -1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.forwardAxis)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveSpeedCap;
     
        DoubleSupplier strafesupp = () -> -1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.strafeAxis)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveSpeedCap;
    
         DoubleSupplier rotatesupp = () -> -1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.rotationAxis)) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.rotationSpeedCap;
    
        m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrain, 
                forwardsupp,
                strafesupp,
                rotatesupp
        ));



    configureBindings();
  }

  public Joystick getPrimaryJoystick(){
    return primaryJoystick;
  }

  private void configureBindings() {}

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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.025);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
