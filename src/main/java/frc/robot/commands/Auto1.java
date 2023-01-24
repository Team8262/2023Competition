import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
// import java.util.ArrayList;
import java.util.List;

public class Auto1 extends SequentialCommandGroup{
    
    public Auto1(Drivetrain drivetrain){

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", 4, 3);

        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));
        eventMap.put("marker3", new PrintCommand("Passed marker 3"));
        eventMap.put("marker4", new PrintCommand("Passed marker 4"));
        eventMap.put("marker5", new PrintCommand("Passed marker 5"));
        

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drivetrain.getpose(),
            drivetrain.setStartPose(Pose2d startPose),
            drivetrain.getKinematics(),
            new PIDConstants(Constants.AUTO_DRIVE_P_CONTROLLER, Constants.AUTO_DRIVE_I_CONTROLLER, Constants.AUTO_DRIVE_D_CONTROLLER),
            new PIDConstants(Constants.AUTO_TURN_P_CONTROLLER, Constants.AUTO_TURN_I_CONTROLLER, Constants.AUTO_TURN_D_CONTROLLER),
            drivetrain.getStates(), 
            eventMap, 
            true,
            drivetrain);

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

    }
}
