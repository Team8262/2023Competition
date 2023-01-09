// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {

  public static final double MAX_VOLTAGE = 13.0;

  SwerveDrivePoseEstimator m_PoseEstimator;
  public static Pose2d DFLT_START_POSE = new Pose2d();

  public static void setStartPose(Pose2d startPose){
        DFLT_START_POSE = startPose;
  }

  
  Pose2d curEstPose = new Pose2d(DFLT_START_POSE.getTranslation(), DFLT_START_POSE.getRotation());
  SwerveModuleState[] states;



  public static TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  
  
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;


    /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );        


  // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  private final AHRS m_navx;

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public boolean auto = false;
  public Drivetrain() {
    m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
  //     tab.getLayout("Front Left Module", BuiltInLayouts.kList)
  //             .withSize(2, 4)
  //             .withPosition(0, 0),
      Mk4SwerveModuleHelper.GearRatio.L2,
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_LEFT_MODULE_STEER_OFFSET
    );

    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
          // tab.getLayout("Front Right Module", BuiltInLayouts.kList)
          //         .withSize(2, 4)
          //         .withPosition(2, 0),
          Mk4SwerveModuleHelper.GearRatio.L2,
          FRONT_RIGHT_MODULE_DRIVE_MOTOR,
          FRONT_RIGHT_MODULE_STEER_MOTOR,
          FRONT_RIGHT_MODULE_STEER_ENCODER,
          FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
          // tab.getLayout("Back Left Module", BuiltInLayouts.kList)
          //         .withSize(2, 4)
          //         .withPosition(4, 0),
          Mk4SwerveModuleHelper.GearRatio.L2,
          BACK_LEFT_MODULE_DRIVE_MOTOR,
          BACK_LEFT_MODULE_STEER_MOTOR,
          BACK_LEFT_MODULE_STEER_ENCODER,
          BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
      //     tab.getLayout("Back Right Module", BuiltInLayouts.kList)
      //             .withSize(2, 4)
      //             .withPosition(6, 0),
          Mk4SwerveModuleHelper.GearRatio.L2,
          BACK_RIGHT_MODULE_DRIVE_MOTOR,
          BACK_RIGHT_MODULE_STEER_MOTOR,
          BACK_RIGHT_MODULE_STEER_ENCODER,
          BACK_RIGHT_MODULE_STEER_OFFSET
    );


    var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));
    var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

    //m_PoseEstimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), DFLT_START_POSE, m_kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,0.02);

    setknownPose(DFLT_START_POSE);
    

    states = new SwerveModuleState[] {new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};


  }

  public void setknownPose(Pose2d in){
    //wants us to reset wheel encoders?
    zeroGyroscope();
    //m_PoseEstimator.resetPosition(in, getGyroscopeRotation());
    curEstPose = in;
  }

  public Pose2d getpose(){
          
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void resetStates(){
    states = new SwerveModuleState[] {new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};
  }

  public void zeroGyroscope() {
      m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
        /*
    if (m_navx.isMagnetometerCalibrated()) {
    // We will only get valid fused headings if the magnetometer is calibrated
    return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
    }*/
    //
    //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //    return Rotation2d.fromDegrees(360-m_navx.getYaw());
    //return Rotation2d.fromDegrees(-(360.0 - m_navx.getYaw()));
    return Rotation2d.fromDegrees(-m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void periodicTelemetry() {
    double FLdegree = Math.toDegrees(m_frontLeftModule.getSteerAngle());
    double FRdegree = Math.toDegrees(m_frontRightModule.getSteerAngle());
    double BLdegree = Math.toDegrees(m_backLeftModule.getSteerAngle());
    double BRdegree = Math.toDegrees(m_backRightModule.getSteerAngle());

    SmartDashboard.putNumber("Front Left Module Angle ", FLdegree);
    SmartDashboard.putNumber("Front Right Module Angle ", FRdegree);
    SmartDashboard.putNumber("Back Left Module Angle ", BLdegree);
    SmartDashboard.putNumber("Back Right Module Angle ", BRdegree);
  }

  @Override
  public void periodic() {
    if(!auto){
      states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);  
    }else{
        //m_PoseEstimator.update(getGyroscopeRotation(), states);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    periodicTelemetry();
  }
}
