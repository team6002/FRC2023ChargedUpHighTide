// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Intake;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CMD_IntakeDropTeleop extends CommandBase {
  SUB_Intake m_intake;
  SUB_FiniteStateMachine m_finiteStateMachine;
  GlobalVariables m_variables;
  CommandXboxController m_driverController;
  boolean m_finished;


  public CMD_IntakeDropTeleop(SUB_Intake p_intake, GlobalVariables p_variables, CommandXboxController p_driverController){
    m_intake = p_intake;
    m_variables = p_variables;
    m_finished = false;
    m_driverController = p_driverController;
    addRequirements(m_intake);
  }

  public void turnIntakeOn() {
    m_intake.setCurrent(IntakeConstants.kIntakeCurrent);
    if (m_variables.getIntakeCommandKey() != -1){
      if(m_variables.getIntakeState() == GlobalConstants.kConeMode){
        if (m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel){
          m_intake.setPower(IntakeConstants.kIntakeDropGroundCone);
        } else {
          m_intake.setPower(IntakeConstants.kIntakeDropCone);
        }
      } else{
        if (m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel){
          m_intake.setPower(IntakeConstants.kIntakeDropGroundCube);
        } else {
          m_intake.setPower(IntakeConstants.kIntakeDropCube);
        }
      }
    } else{
      //nothing TA DA
    }
  }

  @Override
  public void initialize() {
    m_finished = false;

    if (m_variables.getIntakeState() == GlobalConstants.kCubeMode ||
        m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel ||
        m_variables.getAutoDropGood()){
      turnIntakeOn();
      m_finished = true;
    }
  }

  @Override
  public void execute() {
    if (m_finished) {
      return;
    }

    if (m_driverController.leftBumper().getAsBoolean()){
      turnIntakeOn();
      m_finished = true;
    }else {
      m_finished = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_variables.setHasItem(false);
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}