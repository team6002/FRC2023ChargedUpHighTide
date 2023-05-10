// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_FiniteStateMachine;

public class CMD_IntakeDropTeleop extends CommandBase {
  SUB_Intake m_intake;
  SUB_FiniteStateMachine m_finiteStateMachine;
  boolean m_finished;
  GlobalVariables m_variables;
  CommandXboxController m_driverController;


  public CMD_IntakeDropTeleop(SUB_Intake p_intake, GlobalVariables p_variables, CommandXboxController p_driverController){
    m_intake = p_intake;
    m_variables = p_variables;
    m_driverController = p_driverController;
    addRequirements(m_intake);
  }

  private void drop(){
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
    m_intake.setCurrent(IntakeConstants.kIntakeCurrent);
    if (m_variables.getIntakeState() == GlobalConstants.kCubeMode || m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel || m_variables.getDropGood()){
      System.out.println("droped");
      drop();
      m_finished = true;
    }

  }

  @Override
  public void execute() {
    if (m_finished){
      return;
    }
    if (m_driverController.leftBumper().getAsBoolean()) {
      drop();
      m_finished = true;
      System.out.println(";nkafawfl");
      return;
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