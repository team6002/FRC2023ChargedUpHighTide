// readjusts Elements inside to insure its fully inside
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeReadjust extends CommandBase {
  /** Creates a new CMD_IntakeReadjust. */
  SUB_Intake m_intake;
  GlobalVariables m_variables;
  boolean m_finished;
  double m_timer;
  public CMD_IntakeReadjust(SUB_Intake p_intake, GlobalVariables p_variables) {
    m_intake = p_intake;
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = 0;
    m_finished = false;
    if (m_variables.getHasItem() == false){
      m_finished = true;
    }
    m_intake.setCurrent(IntakeConstants.kIntakeCurrent);
    if (m_variables.getIntakeState() == GlobalConstants.kConeMode){
      m_intake.setPower(IntakeConstants.kIntakeForwardPower);
    }else m_intake.setPower(IntakeConstants.kIntakeBackwardPower);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getCurrent() >= IntakeConstants.kIntakeConeDetectedCurrent){
      m_timer += 1;
      if (m_timer >= 30){
        m_finished = true;
      }
    }else m_finished = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
