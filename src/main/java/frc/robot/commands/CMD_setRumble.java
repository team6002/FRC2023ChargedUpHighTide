// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_setRumble extends CommandBase {
  /** Creates a new CMD_setRumble. */
  XboxController m_driveContoller;
  double m_Seconds;
  double m_timer;
  boolean m_finished;
  public CMD_setRumble(XboxController p_driveController, double p_Seconds) {
    m_driveContoller = p_driveController;
    m_Seconds = p_Seconds;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    m_timer = 0;
    m_driveContoller.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_timer += .05;
    if (m_timer > m_Seconds){
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveContoller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
