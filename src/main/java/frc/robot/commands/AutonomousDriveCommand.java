// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
//import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;

public class AutonomousDriveCommand extends CommandBase {                                           
    long startTime; // in milliseconds
    long elapsedTime;
    long elapsedSeconds;

    private final Drivetrain m_drivetrainSubsystem;
    
    double x;
    double y;
    double rotation;
    double speed;
    double seconds;

    public AutonomousDriveCommand(Drivetrain drivetrainSubsystem, double x, double y, double seconds) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.x = x;
        this.y = y;
        this.speed = speed;
        this.seconds = seconds;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() { 
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() { // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        elapsedTime = System.currentTimeMillis() - startTime;
        elapsedSeconds = elapsedTime / 1000;

        m_drivetrainSubsystem.drive(x,y, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return elapsedSeconds >= seconds;
    }
}