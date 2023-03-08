package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class GoToAuto extends SequentialCommandGroup{


    PathPlannerTrajectory trajectory;
    int id;
    Drivetrain drivetrain;
    Pose2d robotPose2d;

    final Pose2d[] TARGETS = {
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0)),
        new Pose2d(0,0, new Rotation2d(0))
    };

    public GoToAuto(int id, Drivetrain drivetrain, Pose2d robotPose2d) {

        this.id = id;
        this.drivetrain = drivetrain;
        this.robotPose2d = robotPose2d;


        //this command generates a path taking into account velocity.

        trajectory = PathPlanner.generatePath(
            new PathConstraints(Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED), 
            false,
            new ArrayList<PathPoint>() {{
                PathPoint.fromCurrentDifferentialState(robotPose2d, drivetrain.getChassisSpeeds());
                new PathPoint(
                    new Translation2d(TARGETS[id].getX(), 
                    TARGETS[id].getY()), 
                    TARGETS[id].getRotation(), 
                    Rotation2d.fromDegrees(0)
                ); // position, heading(direction of travel), holonomic rotation
            }} 
        );
        
        addCommands(
            drivetrain.followTrajectoryCommand(trajectory, false)
        );

    }
}