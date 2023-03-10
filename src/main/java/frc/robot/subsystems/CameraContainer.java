package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraContainer extends SubsystemBase {
    
    private List<PhotonTrackedTarget> targets; 
    private AprilTagFieldLayout aprilTagFieldLayout;
    
    private RobotPoseEstimator robotPoseEstimator;
    private Pose2d currentPose2d = new Pose2d(0,0,new Rotation2d(0));
    private Double poseLatency = 0.0;

    //define all cameras here!
    private PhotonCamera limelightCamera = new PhotonCamera("OV5647");
    private Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));

    

    public CameraContainer() throws IOException {

        //add all your cameras here!
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(limelightCamera, robotToCam));
        
        aprilTagFieldLayout  = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
        robotPoseEstimator.setLastPose(new Pose3d(0,0,0,new Rotation3d(0,0,0)));
    }

    public void periodic() {
        try {

            Pair temp = getEstimatedGlobalPose(currentPose2d);
            currentPose2d = (Pose2d) temp.getFirst();
            poseLatency = (Double) temp.getSecond();

        } catch(NullPointerException e) {
            System.out.println("NullPointerException: no Apriltag fudicals in sight!");
        } catch(NoSuchElementException f) {
            System.out.println("NoSuchElementException: no Apriltag fudicals in sight!");
        }

        SmartDashboard.putNumber("x (visual)", currentPose2d.getX());
        SmartDashboard.putNumber("y (visual)", currentPose2d.getY());
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
    
    public Pose2d getVisualPose() {
        return currentPose2d;
    }
    
}
