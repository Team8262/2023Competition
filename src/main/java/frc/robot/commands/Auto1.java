package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;




import java.util.HashMap;
// import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;


public class Auto1 extends SequentialCommandGroup{
    
    public Auto1(Drivetrain drivetrain, RobotContainer container){

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "Test_path_1", 
            new PathConstraints(2, 2),
            new PathConstraints(1.5, 2)
           );

        HashMap<String, Command> eventMap = new HashMap<>();
        
        Consumer<Pose2d> thing = drivetrain::resetPose;
        SwerveDriveKinematics kinematics = Constants.KINEMATICS; 
        Consumer<SwerveModuleState[]> joeph = drivetrain.joe;

        eventMap.put("break", Commands.runOnce(drivetrain::enableXstance, drivetrain));


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
        // Command fullAuto = autoBuilderC.fullAuto(pathGroup);
        addCommands(fullAuto);



    }
}
