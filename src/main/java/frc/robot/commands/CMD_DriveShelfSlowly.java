//Drives forward slowly for picking up from shelf
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Intake;

public class CMD_DriveShelfSlowly extends CommandBase {
  /** Creates a new CMD_DriveForwardsSlowly. */
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  CommandXboxController m_driverController;
  double m_drivetimer = 0;
  double m_intaketimer = 0;
  boolean m_finished;
  boolean m_detected;
  public CMD_DriveShelfSlowly(SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, CommandXboxController p_driverController) {
    m_drivetrain = p_drivetrain;
    m_driverController = p_driverController;
    m_intake = p_intake;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    m_detected = false; 
    m_drivetimer = 0;
    m_intaketimer = 0;
    m_drivetrain.drive(.1, 0, 0, false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getCurrent() >= IntakeConstants.kIntakeConeDetectedCurrent) {
      if (m_intaketimer == IntakeConstants.kIntakeDetectedtimer) {
        m_detected = true;
      } else {
        m_intaketimer += 1;
      }
    } else {
      m_detected = false;
      m_intaketimer = 0;
    }

    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      m_finished = true;
      System.out.println("Aborted by driver");
      return;
    }
    m_drivetimer += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drivetimer > 5000) || m_finished || m_detected;
  }
}
