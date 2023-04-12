// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_DriveAlignRetroflective extends CommandBase {
  /** Creates a new CMD_DriveAlignRetroflective. */
  SUB_Limelight m_limelight;
  SUB_Drivetrain m_drivetrain;
  GlobalVariables m_variables;
  double rot;
  double heading_error;
  boolean m_finished;
  double m_offset;
  final double limelightAngleThreshold = LimeLightConstants.klimelightAngleThreshold;
  final double limelightAdjustRotKp = LimeLightConstants.klimelightAdjustRotKp;
  final double limelightAdjustStrafeKp = LimeLightConstants.klimelightAdjustStrafeKp;
  CommandXboxController m_driverController;
  public CMD_DriveAlignRetroflective(SUB_Limelight p_limelight, SUB_Drivetrain p_drivetrain, CommandXboxController p_driverController, GlobalVariables p_variables) {
    m_limelight = p_limelight;
    m_drivetrain = p_drivetrain;
    m_variables = p_variables;
    m_driverController = p_driverController;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getDropLevel() == GlobalConstants.kElevator3rdLevel){
      if (m_variables.getRetoflectiveAlignPosition() == GlobalConstants.kLeftRetroflectiveAlignPosition){
        m_offset = -2;
      }else if (m_variables.getRetoflectiveAlignPosition() == GlobalConstants.kRightRetroflectiveAlignPosition){
        m_offset = 2;
      }else {
        m_offset = 0;
      }
    }else if (m_variables.getDropLevel() == GlobalConstants.kElevator2ndLevel){
      if (m_variables.getRetoflectiveAlignPosition() == GlobalConstants.kLeftRetroflectiveAlignPosition){
        m_offset = -4;
      }else if (m_variables.getRetoflectiveAlignPosition() == GlobalConstants.kRightRetroflectiveAlignPosition){
        m_offset = 4;
      }else {
        m_offset = 0;
      }
    }else {
      m_offset = 0;
    }
    if (m_variables.getIntakeState() == GlobalConstants.kCubeMode || m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel){
      m_finished = true;
    }else {
      m_finished = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      m_finished = true;
      System.out.println("Aborted by driver");
      return;
    }
    if (m_limelight.hasTarget()) {
      heading_error = m_limelight.getTargetTx() + m_offset;
      
        // rot = -strafe_error * limelightAdjustKp;
      if (Math.abs(heading_error) > limelightAngleThreshold) {
        rot = ((-heading_error) * limelightAdjustRotKp) + Math.copySign(LimeLightConstants.klimelightAdjustRotKf, -heading_error);
      }
    }else {
      m_finished = true;
    }
    m_drivetrain.drive(0, 0, rot, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_variables.setRetroflectiveAlignPosition(GlobalConstants.kMiddleRetroflectiveAlignPositon);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(heading_error) < 1.5) || m_finished;
  }
}
