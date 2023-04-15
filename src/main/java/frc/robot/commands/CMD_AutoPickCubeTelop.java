// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_IntakeCamera;

public class CMD_AutoPickCubeTelop extends CommandBase {
  /** Creates a new CMD_AutoPickCube. */
  SUB_Intake m_intake;
  SUB_Elbow m_elbow;
  SUB_IntakeCamera m_intakeCam;
  SUB_Drivetrain m_drivetrain;
  SUB_Elevator m_elevator;
  CommandXboxController m_driverController;
  GlobalVariables m_variables;
  double xSpeed;
  double rot;
  double rotf = 0.001;
  double m_finishTimer;
  boolean m_finished = false;
  public CMD_AutoPickCubeTelop(SUB_IntakeCamera p_intakeCam, SUB_Drivetrain p_drivetrain, CommandXboxController p_driverController, GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_finished = false;
    m_intakeCam = p_intakeCam;
    m_drivetrain = p_drivetrain;
    m_variables = p_variables;
    m_driverController = p_driverController;
    addRequirements(m_drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xSpeed = -.3;
    m_finished = false;
    rot = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeCam.hasTarget()) {
      m_finishTimer = 0;
      double heading_error = m_intakeCam.getTxZero() - m_intakeCam.getTargetTx();

      if (Math.abs(heading_error) > 5) {
        rot = heading_error * 0.00125 + Math.copySign(rotf, heading_error);
      }
      
    }else {
      m_finishTimer += 1;
      if (m_finishTimer >= 50){
        m_finished = true;
      }
    }
    m_drivetrain.drive(xSpeed, 0, rot, false, false);
    if (m_variables.getHasItem() == true){
      m_finished = true;
    }
    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      m_finished = true;
      System.out.println("Aborted by driver");
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false, false);
    System.out.println("DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
