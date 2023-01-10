// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.jumprobotics.swervedrive.SecondSwerveKinematics;
import org.jumprobotics.swervedrive.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class Constants {

    public static final boolean TUNING_MODE = false;

    // FIXME: an empty string uses the default CAN bus; specify the name of the CANivore as appropriate
    public static final String CAN_BUS_NAME = "";

    //Joystick
    public static final int forwardAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 4; //Uncomment for other thing

    //Drivetrain Constants
    public static final double driveSpeedCap = 1; //Percent of max speed
    public static final double rotationSpeedCap = 0.6; 

    // FIXME: check dims on final robot
    
    //The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715;
    //The front-to-back distance between the drivetrain wheels.
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715;

    // FIXME: determine maximum velocities empirically
    /**
     * The maximum velocity of the robot in meters per second.
     *
     * <p>This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
    6380.0
        / 60.0
        / SwerveModuleConstants.DRIVE_GEAR_RATIO
        * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;

    public static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


    // FIXME: tune PID values for auto paths

    public static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
    public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
    public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
    public static final double AUTO_TURN_P_CONTROLLER = 10.0;
    public static final double AUTO_TURN_I_CONTROLLER = 0.0;
    public static final double AUTO_TURN_D_CONTROLLER = 0.0;

    public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0));

    public static final SecondSwerveKinematics YES = new SecondSwerveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0));

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(83.757019);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 13; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(11.947632);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 10; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(69.076538);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 6; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);


}
