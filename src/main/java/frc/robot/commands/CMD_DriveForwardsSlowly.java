//Drives forward slowly for picking up from shelf
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_DriveForwardsSlowly extends CommandBase {
  /** Creates a new CMD_DriveForwardsSlowly. */
  SUB_Drivetrain m_drivetrain;
  double m_timer = 0;
  public CMD_DriveForwardsSlowly(SUB_Drivetrain p_drivetrain) {
    m_drivetrain = p_drivetrain;
    m_timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = 0;
    m_drivetrain.drive(.1, 0, 0, false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    return (m_timer > 100);
  }
}
