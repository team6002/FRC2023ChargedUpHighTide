// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;

public class CMD_SyncElbowPosition extends CommandBase {
  /** Creates a new CMD_SyncElbowPosition. */
  SUB_Elbow m_elbow;
  boolean m_finished;
  double m_timer;
  public CMD_SyncElbowPosition(SUB_Elbow p_elbow) {
    m_elbow = p_elbow;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    m_timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_elbow.getElbowVelocity()) <= 1 ){
      m_timer += 1;
    }else m_timer = 0;
    if (m_timer >= 40){
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Math.abs(m_elbow.getPosition() - m_elbow.getAbsolutePosition()) > 2){
      m_elbow.syncElbowPosition();
      System.out.println(+1);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
