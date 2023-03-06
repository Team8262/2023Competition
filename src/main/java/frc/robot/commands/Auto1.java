package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.RobotContainer;



import java.util.HashMap;
// import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;


public class Auto1 extends SequentialCommandGroup{
    
    public Auto1(Drivetrain drivetrain){

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test Path 1", 4, 3);

        HashMap<String, Command> eventMap = new HashMap<>();
        
        Consumer<Pose2d> thing = drivetrain::resetPose;
        SwerveDriveKinematics kinematics = Constants.KINEMATICS; 
        Consumer<SwerveModuleState[]> joeph = drivetrain.joe;
        RobotContainer container = new RobotContainer();

        eventMap.put("start1", new PrintCommand("Passed marker 1"));
        eventMap.put("place1", new PrintCommand("Passed marker 2"));
        eventMap.put("pick1", new InstantCommand(() ->  container.coneIntake(1.0)));
        eventMap.put("pick2", new PrintCommand("Passed marker 4"));
        eventMap.put("place2", new InstantCommand(() ->  container.coneIntake(0.0)));        
        // eventMap.put("marker5", new PrintCommand("Passed marker 5"));


        SwerveAutoBuilder autoBuilderC = new SwerveAutoBuilder(
            drivetrain::getPose,
            thing,
            new PIDConstants(AUTO_DRIVE_P_CONTROLLER,AUTO_DRIVE_I_CONTROLLER, AUTO_DRIVE_D_CONTROLLER),
            new PIDConstants(AUTO_TURN_P_CONTROLLER, AUTO_TURN_I_CONTROLLER, AUTO_TURN_D_CONTROLLER),
            drivetrain::getStates, 
            eventMap, 
            drivetrain);

        SwerveAutoBuilder autoBuilderS = new SwerveAutoBuilder(
            drivetrain::getPose,
            thing,
            kinematics,
            new PIDConstants(AUTO_DRIVE_P_CONTROLLER,AUTO_DRIVE_I_CONTROLLER, AUTO_DRIVE_D_CONTROLLER),
            new PIDConstants(AUTO_TURN_P_CONTROLLER, AUTO_TURN_I_CONTROLLER, AUTO_TURN_D_CONTROLLER),
            joeph, 
            eventMap, 
            drivetrain);
        

      
      
        Command fullAuto = autoBuilderS.fullAuto(pathGroup);
        
        // public Commmand getauto(){
        //     return fullAuto;
        // }


    }
}
