// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class gayAuto extends SequentialCommandGroup {
  Drivetrain drivetrain;
  /** Creates a new BadAuto. */
  public gayAuto(Drivetrain drivetrain, RobotContainer rc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //ParallelCommandGroup endArm = new ParallelCommandGroup( new gayArmPath(rc.arm, 1, 1), 
                                                   // new SequentialCommandGroup(new WaitCommand(1.5), new InstantCommand(() -> rc.arm.auto=true)));
   // ParallelCommandGroup endArmHome = new ParallelCommandGroup(new gayArmPath(rc.arm, 5,0.5),
                                                    //new SequentialCommandGroup(new WaitCommand(1.5), new InstantCommand(() -> rc.arm.auto = true))
    //);
    // drivetrain = RobotContainer.getDrivetrain();
    addCommands(new AutonomousDriveCommand(drivetrain, 0, 1.3, 0, 2),
                  new AutoBalance(drivetrain)
                );
  }
}
