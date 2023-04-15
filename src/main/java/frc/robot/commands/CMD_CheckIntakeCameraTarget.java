// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_IntakeCamera;

public class CMD_CheckIntakeCameraTarget extends CommandBase {
  /** Creates a new CMD_CheckIntakeCameraTarget. */
  SUB_IntakeCamera m_intakeCam;
  boolean m_finished = false;
  public CMD_CheckIntakeCameraTarget(
    SUB_IntakeCamera p_intakeCam
  ) {
    m_intakeCam = p_intakeCam;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    if (m_intakeCam.hasTarget() == true){
      m_finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeCam.hasTarget() == true){
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
