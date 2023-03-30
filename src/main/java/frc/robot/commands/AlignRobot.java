package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.swing.text.Position;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Vision;

public class AlignRobot extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;
    private final LimelightVision vision;
    PIDController forwardPidController;
    PIDController horizontalPidController;
    PIDController rotationPidController;

    double position;

    double yError = 0;
    double xError = 0;
    
    public boolean DEBUGGING;


    public AlignRobot(Drivetrain drivetrainSubsystem, LimelightVision vision, double position) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.vision = vision;
        this.position = position;
        addRequirements(drivetrainSubsystem);

        //set the pid values
        forwardPidController = new PIDController(0, 0, 0);
        horizontalPidController = new PIDController(0.001, 0, 0);
        rotationPidController = new PIDController(0, 0, 0);

        DEBUGGING = true;

        if(DEBUGGING) {
            ShuffleboardTab robotAlignment = Shuffleboard.getTab("Align_Robot");
            robotAlignment.addDouble("forward PID Position Error: ", () -> forwardPidController.getPositionError());
            robotAlignment.addDouble("horizontal PID Position Error: ", () -> horizontalPidController.getPositionError());
            robotAlignment.addDouble("rotational PID Position Error: ", () -> rotationPidController.getPositionError());
            robotAlignment.addDouble("yError: ", () -> yError);
            robotAlignment.addDouble("xError: ", () -> xError);
           
        }
        
        vision.setPipelineIndex(Constants.REFLECTIVE_TAPE_PIPELINE_INDEX);
        
        
    }

    @Override
    public void execute() {
        System.out.println("aaaaaaaaa");

        yError = (Constants.POLE_HEIGHT - Constants.CAMERA_HEIGHT) / Math.tan(Math.toRadians(vision.getTargetY()));
        xError = yError * Math.tan(Math.toRadians(vision.getTargetY()));

        /*(m_drivetrainSubsystem.drive(
            forwardPidController.calculate(0.0),//(yError, position),
            horizontalPidController.calculate(xError, 0),
            rotationPidController.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), 0)
        );*/
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0,0,0);
    }

}
