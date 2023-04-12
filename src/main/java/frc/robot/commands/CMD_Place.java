// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_Limelight;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;

public class CMD_Place extends CommandBase {
  /** Creates a new CMD_SpagethiPlace. */
  SUB_Elbow m_elbow;
  SUB_Elevator m_elevator;
  SUB_Limelight m_limelight;
  GlobalVariables m_variables;
  CommandXboxController m_driverController;
  boolean m_finished = false;
  int m_dropLevel;
  int m_previousDropLevel;
  boolean m_state;
  boolean m_previousState;
  boolean m_elbowSafe;//if the elbow is able to move down safety
  boolean m_elbowDone;// if the elbow is at the correct spot
  boolean m_elevatorDone;
  boolean m_autodrop;
  double m_wantedElbowPosition;
  double m_wantedElevatorPosition;
  double m_debounceTimer;
  double m_elevatorTimer;
  public CMD_Place(SUB_Elbow p_elbow, SUB_Elevator p_elevator, SUB_Limelight p_limelight, GlobalVariables p_variables, CommandXboxController p_driverController) {
    m_elbow = p_elbow;
    m_elevator = p_elevator;
    m_limelight = p_limelight;
    m_variables = p_variables;
    m_driverController = p_driverController;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  private void m_getConeExtend(){
    m_elbowDone = false;
    m_elbowSafe = false;
    m_elevatorDone = false;
    switch(m_variables.getDropLevel()) {  
      case GlobalConstants.kElevator1stLevel:
        m_wantedElbowPosition = ElbowConstants.kElbowDrop;
        m_wantedElevatorPosition = ElevatorConstants.kElevatorFirstConeLevel;
      break;
      case GlobalConstants.kElevator2ndLevel:
        m_wantedElbowPosition = ElbowConstants.kElbowSecondDrop;
        m_wantedElevatorPosition = ElevatorConstants.kElevatorSecondConeLevel;
      break;
      case GlobalConstants.kElevator3rdLevel:
        m_wantedElbowPosition = ElbowConstants.kElbowDrop;
        m_wantedElevatorPosition = ElevatorConstants.kElevatorThirdConeLevel;
      break;
      default:

      break; 
    }
  }

  private void m_getCubeExtend(){
    m_elbowDone = false;
    m_elbowSafe = false;
    m_elevatorDone = false;
    switch(m_variables.getDropLevel()) {  
      case GlobalConstants.kElevator1stLevel:
      m_wantedElbowPosition = ElbowConstants.kElbowDrop;
      m_wantedElevatorPosition = ElevatorConstants.kElevatorFirstCubeLevel;
      break;
      case GlobalConstants.kElevator2ndLevel:
      m_wantedElbowPosition = ElbowConstants.kElbowSecondDrop;
      m_wantedElevatorPosition = ElevatorConstants.kElevatorSecondCubeLevel;
      break;
      case GlobalConstants.kElevator3rdLevel:
      m_wantedElbowPosition = ElbowConstants.kElbowDrop;
      m_wantedElevatorPosition = ElevatorConstants.kElevatorThirdCubeLevel;
      break;
      default:

      break; 
    }
  }

  private boolean CheckElevator(double p_wantedElevatorPosition){
    boolean m_ElevatorThere = false;
    m_ElevatorThere = Math.abs(m_elevator.getPosition() - p_wantedElevatorPosition) < ElevatorConstants.kElevatorTolerance;
    if (m_ElevatorThere == true){
      m_elbowSafe = true;
    }else m_elbowSafe = false;
    return m_ElevatorThere;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autodrop = m_variables.getAutoDrop();
    m_debounceTimer = 0;
    m_finished = false;
    if (m_variables.getIntakeState() == GlobalConstants.kConeMode){
      m_getConeExtend();
    }else {
      m_getCubeExtend();
    }
    m_previousState = m_state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_debounceTimer += 1;
    m_elevatorTimer += 1;
    m_state = m_variables.getIntakeState();
    m_dropLevel = m_variables.getDropLevel();
    CheckElevator(m_wantedElevatorPosition);
    if (m_debounceTimer > 50){
      if (m_driverController.leftBumper().getAsBoolean()){
        m_finished = true;
      }else {
        m_finished = false;
      }
    }
    if (m_dropLevel != m_previousDropLevel){
      m_elevatorTimer = 0;
      m_elbow.setReference(ElbowConstants.kElbowLifted);
      if (m_variables.getIntakeState() == GlobalConstants.kConeMode){
        m_getConeExtend();
      }else {
        m_getCubeExtend();
      }
    }  
    if (m_elbowSafe == true) {
      if (m_elbowDone == false){
        m_elbow.setReference(m_wantedElbowPosition);
        m_elbowDone = true;
      }
    }else {

    }  
    if (m_elevatorTimer > 25){
      if (m_elevatorDone == false){
        m_elevator.setReference(m_wantedElevatorPosition);
        m_elevatorDone = true;
      }else{
      }
    }
    if (m_autodrop && m_elbowDone && m_elevatorDone && m_limelight.hasTarget()){
      m_finished = true;
    }
  m_previousDropLevel = m_dropLevel;  
  m_previousState = m_state; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Placed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
