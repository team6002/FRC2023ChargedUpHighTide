//NOTE THE RADIO IS LOOSING  CONNECTION
package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Constants.AutoAlignConstants.AlignPosition;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  private final SUB_Blinkin m_blinkin = new SUB_Blinkin();
  private final SUB_PixyCam m_pixyCam = new SUB_PixyCam();
  private final SUB_FiniteStateMachine m_finiteStateMachine = new SUB_FiniteStateMachine();
  private final GlobalVariables m_variables = new GlobalVariables();
  private final SUB_IntakeCamera m_intakeCam = new SUB_IntakeCamera();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverControllerHI = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final BooleanSupplier HasItem = () -> m_variables.getHasItem();
  private final BooleanSupplier IntakeState = () -> m_variables.getIntakeState();
  private final BooleanSupplier GroundLevel = () -> (m_variables.getDropLevel() == 1);

  private final String buildInfoFilename = "buildInfo.txt";
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    //changes blinking codes hopefully >I<
    m_blinkin.setDefaultCommand(new CMD_BlinkinSetIntakeSignal(m_blinkin, m_variables));
    m_limelight.setDefaultCommand(new ParallelCommandGroup(
      new CMD_CheckShelfDistance(m_limelight, m_driverControllerHI),
      new CMD_selectLimelightPipeline(m_limelight, m_variables))
    );
    //this drives
    m_drivetrain.setDefaultCommand(new CMD_Drive(m_drivetrain, m_driverController, m_limelight, m_intakeCam));
    // m_drivetrain.setDefaultCommand(new CMD_setSPEEDtoModules(m_drivetrain, m_driverController));//ONLY FOR TESTING
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
    // the main command that cycles through the robot sequence
    m_driverController.leftBumper().onTrue(getCycleCommand);  
    //This command send you all the way back to the intake sequence
    m_driverController.rightBumper().onTrue(new SequentialCommandGroup(
      new CMD_SetStage(m_variables, GlobalConstants.kIntakeStage),
      new CMD_setHasItem(m_variables, false),
      new CMD_Stow(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)
    ));
    //changes intake 
    m_driverController.b().onTrue(new SequentialCommandGroup(
      new CMD_ToggleIntakeState(m_variables),
      new CMD_BlinkinSetIntakeSignal(m_blinkin, m_variables)
    ));
    
    m_driverController.a().onTrue(new CMD_AutoPickCubeTelop(m_intakeCam, m_drivetrain, m_driverController, m_variables));
      //new CMD_DriveShelfSlowly(m_drivetrain, m_intake, m_driverController));
    // toggle which pick up mode it will do (Ground or shelf)
    m_driverController.x().onTrue(new CMD_TogglePickMode(m_variables));

    //resets gyro to absoulute encoders
    m_driverController.povLeft().onTrue(new SequentialCommandGroup(
      new CMD_ResetGyro(m_drivetrain)
    ));
    
    m_driverController.povRight().onTrue(new ParallelCommandGroup(
      new CMD_ElevatorSetPosition(m_elevator, ElevatorConstants.kElevatorFirstConeLevel),
      new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowLifted),  
      new CMD_IntakeDropAuto(m_intake, m_variables)
      ));
    
    m_driverController.povDown().onTrue(new CMD_Home(m_intake, m_elbow, m_elevator));
    //Just in case the operator is unable toc perform
    m_driverController.povUp().onTrue(new CMD_ToggleDropLevel(m_variables));

    
    /* ==================OPERATOR CONTROLS GO================== */
    //first level
    m_operatorController.a().onTrue(new CMD_setDropLevel(m_variables, GlobalConstants.kElevator1stLevel));
    //second level
    m_operatorController.b().onTrue(new CMD_setDropLevel(m_variables, GlobalConstants.kElevator2ndLevel));
    //third level
    m_operatorController.y().onTrue(new CMD_setDropLevel(m_variables, GlobalConstants.kElevator3rdLevel));
    //changes intake state
    m_operatorController.x().onTrue(new CMD_ToggleIntakeState(m_variables));
    // offsets the auto align to the left
    m_operatorController.leftBumper().onTrue(new CMD_setRetroflectiveAlignPosition(m_variables, GlobalConstants.kLeftRetroflectiveAlignPosition));
    // offsets the auto align to the right
    m_operatorController.rightBumper().onTrue(new CMD_setRetroflectiveAlignPosition(m_variables, GlobalConstants.kRightRetroflectiveAlignPosition));
    // set the offset back to the middle
    m_operatorController.back().onTrue(new CMD_setRetroflectiveAlignPosition(m_variables, GlobalConstants.kMiddleRetroflectiveAlignPositon));
    // sets autodrop true
    m_operatorController.povLeft().onTrue(new CMD_setAutoDrop(m_variables, true));
    // sets autodrop false
    m_operatorController.povRight().onTrue(new CMD_setAutoDrop(m_variables, false));
    //KIRBY go WHOAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH
    m_operatorController.povUp().onTrue(new CMD_IntakeElementJanky(m_intake, m_elbow, m_variables, m_driverController));

    /* ==================OPERATOR CONTROLS END================== */
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getFullLinkDividerRed() {
    return new AUTO_PPFullLinkDividerRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_intakeCam, m_driverController);
  }

  public Command getFullLinkDividerBlue() {
    return new AUTO_PPFullLinkDividerBlue(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_intakeCam, m_driverController);
  }

  public Command getFullLinkSpeedBumpBlue() {
    return new AUTO_PPFullLinkSpeedBumpBlue(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_intakeCam, m_driverController);
  }

  public Command getFullLinkSpeedBumpRed() {
    return new AUTO_PPFullLinkSpeedBumpRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_intakeCam, m_driverController);
  }
  
  public Command getPP2BalanceBlue() {
    return new AUTO_PP2BalanceBlue(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  }

  public Command getPP2SpeedBumpRed() {
    return new AUTO_PP2SpeedBumpRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  }

  public Command getLinkRunRed() {
    return new AUTO_FullLinkRunRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  }

  public Command getBalanceStation() {
    return new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_variables, m_intakeCam, m_driverController);
  }

  public Command getBalanceStationNoPick() {
    return new AUTO_BalanceStationNoPick(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_variables, m_driverController);
  }

  public Command getPlace(){
    return new AUTO_Place(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_variables, m_intakeCam, m_driverController);
  }
  public Command getNothing() {
    return new AUTO_NOTHING();
  }
  
  public void zeroHeading(){
    m_drivetrain.zeroHeading();
  }

  public void setAutoKey(int p_key){
    m_variables.setAutoKey(p_key);
  }
  public void SubsystemsInit(){
    m_intakeCam.init();
    m_elbow.elbowInit();
    m_elevator.elevatorInit();
    m_intake.intakeInit();
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
      // Map.entry(GlobalConstants.kGroundBackConeDown, new CMD_GroundConeDownIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_ShelfConeIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_ShelfCubeIntake(m_intake, m_elbow, m_elevator, m_finiteStateMachine))
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
    this::getIntakeType
  );

  
  public final Command getConeLevelCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kElevator1stLevel, new CMD_Place1stLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kElevator2ndLevel, new CMD_Place2ndConeLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kElevator3rdLevel, new CMD_Place3rdConeLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables))
    ), 
    this::getDropLevel
  );

  public final Command getCubeLevelCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kElevator1stLevel, new CMD_Place1stLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kElevator2ndLevel, new CMD_Place2ndCubeLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kElevator3rdLevel, new CMD_Place3rdCubeLevel(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables))
    ), 
    this::getDropLevel
  );

  public final Command getConeElbowDropCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kElevator1stLevel, new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowDrop).withTimeout(1)),
      Map.entry(GlobalConstants.kElevator2ndLevel, new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowSecondDrop).withTimeout(1)),
      Map.entry(GlobalConstants.kElevator3rdLevel, new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowDrop).withTimeout(1))
    ), 
    this::getDropLevel
  );
  
  public final Command getCubeElbowDropCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kElevator1stLevel, new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowDrop).withTimeout(1)),
      Map.entry(GlobalConstants.kElevator2ndLevel, new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowDrop).withTimeout(1)),
      Map.entry(GlobalConstants.kElevator3rdLevel, new CMD_ElbowSetPosition(m_elbow, ElbowConstants.kElbowDrop).withTimeout(1))
    ), 
    this::getDropLevel
  );
  

  public final Command getCycleCommand =
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kIntakeStage, new SequentialCommandGroup(
        new CMD_selectIntakeCommandKey(m_intake, m_variables),
        getIntakeCommand,
        new CMD_IntakeElementJanky(m_intake, m_elbow, m_variables, m_driverController),
        getHoldCommand,
        new CMD_SetStage(m_variables, GlobalConstants.kDropStage)
      )),
      // Map.entry(GlobalConstants.kExtendStage,new SequentialCommandGroup(
      // new ParallelCommandGroup(
      //   new CMD_DriveAlignRetroflective(m_limelight, m_drivetrain, m_driverController, m_variables).withTimeout(2),
      //   new CMD_IntakeExtraHold(m_intake, m_variables),
      //   new ConditionalCommand(
      //     getConeLevelCommand, 
      //     getCubeLevelCommand, 
      //     IntakeState)
      // ),
      // new CMD_SetStage(m_variables, GlobalConstants.kDropStage)
      // )),
      Map.entry(GlobalConstants.kDropStage, new SequentialCommandGroup(
        // new ParallelCommandGroup(
        new ConditionalCommand(
          new CMD_Place(m_elbow, m_elevator, m_limelight, m_variables, m_driverController),
          getCubeLevelCommand,
          IntakeState),
        new CMD_DriveAlignRetroflective(m_limelight, m_drivetrain, m_driverController, m_variables).withTimeout(3),
        new CMD_IntakeDropTeleop(m_intake, m_variables, m_driverController),
        new WaitCommand(.2),
        new CMD_Stow(m_intake, m_elbow, m_elevator, m_finiteStateMachine, m_variables),
        new CMD_SetStage(m_variables, GlobalConstants.kIntakeStage)
      ))
    ), 
    this::getRobotStage
  );

  public void loadSoftwareBuildInfo() {
    File deployDir = Filesystem.getDeployDirectory();
    File buildInfoFile = new File(deployDir, buildInfoFilename);

    try {
      Scanner sc = new Scanner(buildInfoFile);
      String latestCommit = sc.nextLine();
      String buildDate = sc.nextLine();

      sc.close();

      SmartDashboard.putString("BuildInfo", latestCommit + " [" + buildDate + "]");
    } catch (FileNotFoundException ex) {
      System.out.println("ERROR: CANNOT READ BUILD INFO");
    }
  }
}
