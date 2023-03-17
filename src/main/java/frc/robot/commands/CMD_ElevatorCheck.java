//checks the location of the elevator
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorCheck extends CommandBase {
  /** Creates a new CMD_ElevatorCheck. */
  SUB_Elevator m_elevator;
  double m_position;
  boolean m_finished;
  public CMD_ElevatorCheck(SUB_Elevator p_elevator, double p_position) {
    m_elevator = p_elevator;
    m_position = p_position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_finished = (m_elevator.getPosition() > m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
