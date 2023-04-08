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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BadAuto extends SequentialCommandGroup {
  /** Creates a new BadAuto. */
  public BadAuto(RobotContainer rc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    ParallelCommandGroup endArm = new ParallelCommandGroup( new gayArmPath(rc.arm, 1, 1), 
                                                    new SequentialCommandGroup(new WaitCommand(1.5), new InstantCommand(() -> rc.arm.auto=true)));
    ParallelCommandGroup endArmHome = new ParallelCommandGroup(new gayArmPath(rc.arm, 5,0.5),
                                                    new SequentialCommandGroup(new WaitCommand(1.5), new InstantCommand(() -> rc.arm.auto = true))
    );
    addCommands(new InstantCommand(() -> rc.getDrivetrain().zeroGyroscope()),
    new InstantCommand(() -> rc.end.setSpeed(0)),
                new gayArmPathTime(rc.arm, rc.end, 1,0.5,8),
                new ParallelCommandGroup(new HoldArm(rc.arm, -.200, -.581, 2), new InstantCommand(() -> rc.end.setSpeed(0.3))),
                new ParallelCommandGroup(new HoldArm(rc.arm, -0.2, -0.581, 3), new InstantCommand(() -> rc.end.setSpeed(-0.7))),
                new InstantCommand(() -> rc.end.setSpeed(0)),
                new gayArmPath(rc.arm, 5, .5),
                new AutonomousDriveCommand(rc.getDrivetrain(), -0.7, 0, 0, 4.2) //Positive y goes right
                );
  }
}
