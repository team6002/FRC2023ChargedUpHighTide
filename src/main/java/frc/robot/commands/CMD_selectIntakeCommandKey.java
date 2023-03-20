// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.SUB_Intake;

public class CMD_selectIntakeCommandKey extends CommandBase {
  /** Creates a new CMD_selectIntakeCommandKey. */
  SUB_Intake m_intake;
  GlobalVariables m_variables;
  boolean state;
  public CMD_selectIntakeCommandKey(SUB_Intake p_intake, GlobalVariables p_variables) {
    m_intake = p_intake;
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = m_variables.getIntakeState();
    switch (m_variables.getPickMode()){
    case GlobalConstants.kPickForwardsShelfMode:
      m_variables.setIntakeCommandKey((state == GlobalConstants.kConeMode) ? GlobalConstants.kShelfForwardsCone : GlobalConstants.kShelfForwardsCube);
      break;
    case GlobalConstants.kPickBackGroundMode:
      m_variables.setIntakeCommandKey((state == GlobalConstants.kConeMode) ? GlobalConstants.kGroundBackConeUpright : GlobalConstants.kGroundBackCube);
      break;
    case GlobalConstants.kPickConeDownMode:
      m_variables.setIntakeCommandKey((state == GlobalConstants.kConeMode) ? GlobalConstants.kGroundBackConeDown : GlobalConstants.kGroundBackCube);
      break;        
      }
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
