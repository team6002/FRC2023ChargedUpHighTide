//when you come in from the scoring side
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SUB_Drivetrain;


public class CMD_AdjustBalanceInside extends CommandBase {
  SUB_Drivetrain m_drivetrain;
  Timer m_timer = new Timer();
  double m_timeLimit;
  public CMD_AdjustBalanceInside(SUB_Drivetrain p_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
    if (Math.abs(m_drivetrain.getRoll()) < 5){
      m_timeLimit = Math.abs(m_drivetrain.getRoll() * 0.03);  
    } else{ 
    m_timeLimit = Math.abs(m_drivetrain.getRoll() * 0.06);  
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_drivetrain.drive(-Math.copySign(0.17, m_drivetrain.getRoll()), 0, 0, true, false);
    //use the Navx if availbe

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, interrupted, interrupted);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > m_timeLimit);
  }
}