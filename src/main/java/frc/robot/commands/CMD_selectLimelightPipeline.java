// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_selectLimelightPipeline extends CommandBase {
  /** Creates a new CMD_selectLimelightPipeling. */
  SUB_Limelight m_limelight;
  GlobalVariables m_variables;
  boolean m_item;//if it has a element or not
  boolean m_previousItem;
  public CMD_selectLimelightPipeline(SUB_Limelight p_limelight, GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = p_limelight;
    m_variables = p_variables;
  }

  public void changePipeline(){
    if (m_item == true){
      m_limelight.setPipeline(LimeLightConstants.kCone2ndPipelineId);;
    }else {
      if (m_variables.getDropLevel() == GlobalConstants.kElevator2ndLevel){
        m_limelight.setPipeline(LimeLightConstants.kCone2ndPipelineId);
      }else {
        m_limelight.setPipeline(LimeLightConstants.kCone3rdPipelineId);
      }
    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    changePipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_item = m_variables.getHasItem(); 
    if (m_item != m_previousItem){
      changePipeline();
    }
    m_previousItem = m_item;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
