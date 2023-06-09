// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_CheckShelfDistance extends CommandBase {
  /** Creates a new CMD_CheckShelfDistance. */
  SUB_Limelight m_limelight;
  XboxController m_driverController;
  double m_timer;
  double m_heartbeat;
  double m_TagID;
  double m_PreivousTagID;
  public CMD_CheckShelfDistance(SUB_Limelight p_limelight, XboxController p_driverContoller) {
    m_limelight = p_limelight;
    m_driverController = p_driverContoller;
    addRequirements(m_limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  private void RumbleCheck(){
    if (m_limelight.getTargetX() >= -1.8){
      m_timer += 1;
      m_driverController.setRumble(RumbleType.kBothRumble, 1);
    }else{
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
      m_timer = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("timer", m_timer);
    m_TagID = m_limelight.getTargetID();
    if (m_TagID == 4 || m_TagID == 5){    
      if (m_timer <= 35){
        RumbleCheck();
      }else {
        m_driverController.setRumble(RumbleType.kBothRumble, 0);
        if (m_timer >= 200){
          m_timer = 0;
        }
      }
    }else {
      m_timer = 0;
    }
    if (m_TagID != m_PreivousTagID){
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
    m_PreivousTagID = m_TagID;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      m_driverController.setRumble(RumbleType.kBothRumble, 0);    
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
