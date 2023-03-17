//NOTE THE RADIO IS LOOSING  CONNECTION
package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SUB_Drivetrain m_drivetrain = new SUB_Drivetrain();
  private final SUB_Limelight m_limelight = new SUB_Limelight();
  private final AUTO_Trajectories m_trajectories = new AUTO_Trajectories(m_drivetrain);
  private final SUB_Elevator m_elevator = new SUB_Elevator();
  private final SUB_Elbow m_elbow = new SUB_Elbow();
  private final SUB_Intake m_intake = new SUB_Intake();
  private final SUB_FiniteStateMachine m_finiteStateMachine = new SUB_FiniteStateMachine();
  private final GlobalVariables m_variables = new GlobalVariables();
  private final SUB_Blinkin m_blinkin = new SUB_Blinkin();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_drivetrain.setDefaultCommand(new CMD_Drive(m_drivetrain, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommandManual() {
  //   return 
  //     new AUTO_CubeRunRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController); 
  //     // new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_variables, m_driverController);
  // }

  // public Command getCubeRunBlue() {
  //   return new AUTO_CubeRunBlue(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  // }

  // public Command getCubeRunRed() {
  //   return new AUTO_CubeRunRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  // }

  // public Command getBalanceStation() {
  //   return new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_variables, m_driverController);
  // }
  
  public void zeroHeading(){
    m_drivetrain.zeroHeading();
  }

  public void setAutoKey(int p_key){
    m_variables.setAutoKey(p_key);
  }
  public void SubsystemsInit(){
    m_elbow.elbowInit();
    m_elevator.elevatorInit();
  }

  private int getIntakeType() {
    return m_variables.getIntakeCommandKey();
  }

  private int getDropLevel(){
    return m_variables.getDropLevel();
  }
  private int getRobotStage(){
    return m_variables.getStage();
  }

  private boolean getIntakeState(){
    return m_variables.getIntakeState();
  }

  private int getAutonomousCommandKey(){
    return m_variables.getAutoKey();
  }
  
  public final Command getAutonomusCommand =
  new SelectCommand(
    Map.ofEntries(
      // Map.entry(AutoConstants.kBalanceStationKey, new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_variables, m_driverController)),
      // Map.entry(AutoConstants.kCubeRunKey, new AUTO_CubeRun(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController))
      Map.entry(AutoConstants.kBalanceStationKey, new PrintCommand("1")),
      Map.entry(AutoConstants.kCubeRunKey, new PrintCommand("2"))
    ), 
    this::getAutonomousCommandKey
  );

  public final Command getIntakeCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kGroundBackCube, new CMD_GroundCubeIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kGroundBackConeUpright, new CMD_GroundConeUprightIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kGroundBackConeDown, new CMD_GroundConeDownIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_ShelfIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_ShelfIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine))
    ), 
    this::getIntakeType
  );

  public final Command getHoldCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kGroundBackCube, new CMD_GroundHold(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kGroundBackConeUpright, new CMD_GroundHold(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kGroundBackConeDown, new CMD_GroundHold(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_ShelfHold(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_ShelfHold(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables))
    ), 
    this::getIntakeState
  );

  
  public final Command getLevelCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kElevator1stLevel, new CMD_Place1stLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kElevator2ndLevel, new CMD_Place2ndLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kElevator3rdLevel, new CMD_Place3rdLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables))
    ), 
    this::getDropLevel
  );
  
  public final Command getCycleCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kIntakeStage, new SequentialCommandGroup(
        getIntakeCommand,
        new CMD_IntakeElement(m_intake, m_variables, m_driverController),
        getHoldCommand,
        new CMD_SetStage(m_variables, GlobalConstants.kExtendStage)
      )),
      Map.entry(GlobalConstants.kExtendStage,new SequentialCommandGroup(
      getLevelCommand,
      new CMD_SetStage(m_variables, GlobalConstants.kDropStage)
      )),
      Map.entry(GlobalConstants.kDropStage, new SequentialCommandGroup(
        new CMD_IntakeDrop(m_intake, m_variables),
        new WaitCommand(.2),
        new CMD_Stow(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables),
        new CMD_SetStage(m_variables, GlobalConstants.kIntakeStage)
      ))
    ), 
    this::getRobotStage
  );

}
