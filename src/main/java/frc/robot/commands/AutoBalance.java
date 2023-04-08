// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PIDCommand {
  /** Creates a new AutoBalance. */
  private Drivetrain dt;

  public AutoBalance(Drivetrain dt) {

    super(
        // The controller that the command will use
        new PIDController(3, 0, 0.2),
        // This should return the measurement
        () -> dt.getGyroIOInputs().connected ? dt.getGyroIOInputs().pitchPosition : 0,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          dt.drive(-0.01*output,0, 0);
        });

        
    addRequirements(dt);
    this.dt = dt;
    dt.zeroGyroscope();

    //This might do something
    getController().setTolerance(1);

    // Configure additional PID options by calling `getController` here.
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    dt.drive(0, 0, 0);
    dt.enableXstance();
    System.out.println("done");
  }
}
