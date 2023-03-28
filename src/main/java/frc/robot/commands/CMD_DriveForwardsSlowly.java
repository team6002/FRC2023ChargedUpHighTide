//Drives forward slowly for picking up from shelf
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_DriveForwardsSlowly extends CommandBase {
  /** Creates a new CMD_DriveForwardsSlowly. */
  SUB_Drivetrain m_drivetrain;
  CommandXboxController m_driverController;
  double m_timer = 0;
  boolean m_finished;
  public CMD_DriveForwardsSlowly(SUB_Drivetrain p_drivetrain, CommandXboxController p_driverController) {
    m_drivetrain = p_drivetrain;
    m_driverController = p_driverController;
    m_timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    m_timer = 0;
    m_drivetrain.drive(.1, 0, 0, false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      m_finished = true;
      System.out.println("Aborted by driver");
      return;
    }
    m_timer += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer > 50) || m_finished;
  }
}
