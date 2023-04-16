/*this is for setting a set speed and testing Swerve Modules
  DO NOT USE FOR NORMAL USE
*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_setSPEEDtoModules extends CommandBase {
  /** Creates a new CMD_setSPEEDtoModules. */
  SUB_Drivetrain m_drivetrain;
  CommandXboxController m_driverController;
  double power;
  public CMD_setSPEEDtoModules(SUB_Drivetrain p_drivetrain, CommandXboxController p_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_driverController = p_driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setDrivePower(m_driverController.getLeftY());
    // m_drivetrain.setTurnPower(m_driverController.getLeftY());
    // m_drivertrain.setDrivePower(power);
    // m_drivetrain.setTurnPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
