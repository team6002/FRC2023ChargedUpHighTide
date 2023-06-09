// This is to swap between pickup mode of downed cones and upright cones
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeElementJanky extends CommandBase {
  /** Creates a new CMD_IntakeCheck. */
  SUB_Intake m_intake;
  GlobalVariables m_variables;
  SUB_Elbow m_elbow;
  boolean m_detected = false;
  boolean m_pressed = false;
  double m_timer = 0;
  boolean m_previousState;
  boolean m_state;
  CommandXboxController m_driverController;

  public CMD_IntakeElementJanky(SUB_Intake p_intake, SUB_Elbow p_elbow, GlobalVariables p_variables, CommandXboxController p_driverController) {
    m_driverController = p_driverController;
    m_intake = p_intake;
    m_elbow = p_elbow;
    m_timer = 0;
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_intake);
  }

  private void setIntakeMode() {
    if (m_variables.getIntakeCommandKey() != -1){
      if (m_state == GlobalConstants.kConeMode){
        if (m_variables.getIntakeCommandKey() == GlobalConstants.kGroundBackConeDown || m_variables.getIntakeCommandKey() == GlobalConstants.kGroundBackConeUpright){
          /* Cone ground */
          m_intake.setPower(IntakeConstants.kIntakeConeDownPower);
        } else {
          /* Cone shelf */
          m_intake.setPower(IntakeConstants.kIntakeForwardPower);
        }
      } else if (m_state == GlobalConstants.kCubeMode) {
        /* Cube shelf and ground */
        m_intake.setPower(-IntakeConstants.kIntakeForwardPower);
      }
    } else{
      //not nothing cancels command
      m_pressed = true;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = 0;
    m_pressed = false;
    m_detected = false;
    m_state = m_variables.getIntakeState();
    m_intake.setCurrent(IntakeConstants.kIntakeCurrent);

    setIntakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driverController.back().getAsBoolean()){
      m_pressed = true;
    }

    if (m_intake.getCurrent() >= IntakeConstants.kIntakeConeDetectedCurrent) {
      if (m_timer == IntakeConstants.kIntakeDetectedtimer) {
        m_detected = true;
      } else {
        m_timer += 1;
      }
    } else {
      m_detected = false;
      m_timer = 0;
    }
    m_state = m_variables.getIntakeState();
    if (m_state != m_previousState){
        setIntakeMode();
    }
  
    m_previousState = m_state;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_pressed == true){
      m_variables.setStage(GlobalConstants.kIntakeStage);
      m_variables.setHasItem(false);
    }else {
      m_variables.setStage(GlobalConstants.kExtendStage);
      m_variables.setHasItem(true);
    }
    m_detected = false;
    m_pressed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_detected || m_pressed;
  }

}
