// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.jumprobotics.swervedrive.SecondSwerveKinematics;
import org.jumprobotics.swervedrive.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class Constants {

    //Obviously tune it if you need to tune it...
    public static final boolean TUNING_MODE = false;

    // FIXME: an empty string uses the default CAN bus; specify the name of the CANivore as appropriate
    public static final String CAN_BUS_NAME = "swerve";

    //Joystick
    public static final int forwardAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 4; //Uncomment for other thing

    //Drivetrain Constants
    public static double driveSpeedCap = 0.8; //Percent of max speed
    public static final double rotationSpeedCap = 0.6; 

    // FIXME: check dims on final robot
    
    //The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6096;
    //The front-to-back distance between the drivetrain wheels.
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6096;

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

    //FIXME: calculate the correct acceleration
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

    // FIXME: tune PID values for auto paths

    public static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
    public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
    public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
    public static final double AUTO_TURN_P_CONTROLLER = 10.0;
    public static final double AUTO_TURN_I_CONTROLLER = 0.0;
    public static final double AUTO_TURN_D_CONTROLLER = 0.0;

    //Intake Constants
    public static final int  intakeMotor = 29;

    //Arm Subsystem
    public static final double BASE_LINK_LENGTH = 0.8128; //meters
    public static final double UPPER_LINK_LENGTH = 0.635+0.1778;//double check this one

    public static final double BASE_VOLTAGE_COMPENSATION = 1.5;//Voltage required to keep base horizontal - should probably test
    public static final double UPPER_VOLTAGE_COMPENSATION = 1.5; //Voltage required to keep upper horizontal
    
    public static final double MAX_TRANSLATION_SPEED = 1; //meters per second
    public static final double MAX_ANGULAR_SPEED = 0.1; //Radians per execution loop

    //Used to convert from final arm angle to motor angle... so 
    public static final double BASE_LINK_GEAR_RATIO = 5*4*4*30/15;
    public static final double UPPER_LINK_GEAR_RATIO = 5*4*4*42/22;

    public static final double BASE_LINK_VELOCITY_P_CONTROLLER = 7.0;
    public static final double BASE_LINK_VELOCITY_I_CONTROLLER = 0.0;
    public static final double BASE_LINK_VELOCITY_D_CONTROLLER = 0.5;
    public static final double BASE_LINK_VELOCITY_F_CONTROLLER = 0.0;

    public static final double BASE_LINK_MAX_VELOCITY = 0.0; //RPM
    public static final double BASE_LINK_MAX_ACCELERATION = 0.0;

    public static final double UPPER_LINK_VELOCITY_P_CONTROLLER = 8.0;
    public static final double UPPER_LINK_VELOCITY_I_CONTROLLER = 0.0;
    public static final double UPPER_LINK_VELOCITY_D_CONTROLLER = 0.2;
    public static final double UPPER_LINK_VELOCITY_F_CONTROLLER = 0.0;

    public static final double UPPER_LINK_MAX_VELOCITY = 50.0; //RPM
    public static final double UPPER_LINK_MAX_ACCELERATION = 50.0;

    //End Constants
    public static final int END_BOTTOM = 22;
    public static final int END_TOP = 21;
    public static final double MAX_END_CURRENT = 40; //Amps

    
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

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; // FIXME Set front left steer encoder ID
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 88.5;//Math.toRadians(88.5); //ID 0
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 157;//Math.toRadians(88.5); //ID 0

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 13; // FIXME Set back right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set back right steer encoder ID

    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 16.87;//Math.toRadians(16.87); //ID 1
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 198 - 180;//1.1;//15.9;//Math.toRadians(16.87); //ID 1

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 6; // FIXME Set back left steer encoder ID
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = 67.32;//Math.toRadians(67.32); //ID 2
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 80.59;//Math.toRadians(67.32); //ID 2

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 352.08;//Math.toRadians(352.08); //ID 3
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 85 + 180;//-149;//-125.6;//0.43-90;//Math.toRadians(352.08); //ID 3

    public static final Transform3d LIMELIGHT_POSITION = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0,0,0) //one of the values might need to be flipped by 180 because the limelight is mounted on the back
    );

    public static final int REFLECTIVE_TAPE_PIPELINE_INDEX = 1;

    //in meters
    public static final double CAMERA_HEIGHT = 0.43;

    public static final double POLE_HEIGHT = 0.56+0.05;

    public static String cameraName = "OV5647";

    public static Transform3d robotToCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));


}
