// this selects the proper align position based on if we have an item or not and what has been selected
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.AutoAlignConstants.AlignPosition;

public class CMD_SelectAlignPosition extends CommandBase {
  /** Creates a new CMD_SelectAlignPosition. */
  GlobalVariables m_variables;
  public CMD_SelectAlignPosition(GlobalVariables p_variables) {
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getHasItem() == true){
      switch (m_variables.getPlaceAlignPosition()){
        case GlobalConstants.kLeftPlacePosition:
          m_variables.setAlignPosition(AlignPosition.LEFTSCORE);
          break;

        case GlobalConstants.kMiddlePlacePosition:
          m_variables.setAlignPosition(AlignPosition.MIDDLESCORE);
          break;

        case GlobalConstants.kRightPlacePosition:
          m_variables.setAlignPosition(AlignPosition.RIGHTSCORE);
          break;
      }
    }else {
      switch (m_variables.getPickMode()){
        case GlobalConstants.kLeftPickPosition:
          m_variables.setAlignPosition(AlignPosition.LEFTSHELF);
          break;
      
        case GlobalConstants.kRightPickPosition:
          m_variables.setAlignPosition(AlignPosition.RIGHTSHELF);
          break;
      }

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
