// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CMD_setInitialOdometeryHolonomic;
import frc.robot.subsystems.SUB_Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_Test extends SequentialCommandGroup {
  /** Creates a new AUTO_PPFullLinkDivider. */
  AUTO_Trajectories m_trajectories;
  public AUTO_Test(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain) {
    m_trajectories = p_trajectories;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_setInitialOdometeryHolonomic(p_drivetrain, m_trajectories.test),
      m_trajectories.followTrajectoryCommand(m_trajectories.test)
    );
  }
}
