// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.GlobalVariables;
import frc.robot.commands.CMD_AdjustBalanceInside;
import frc.robot.commands.CMD_CheckOnCharge;
import frc.robot.commands.CMD_GroundCubeIntake;
import frc.robot.commands.CMD_GroundHold;
import frc.robot.commands.CMD_IntakeDropAuto;
import frc.robot.commands.CMD_IntakeHold;
import frc.robot.commands.CMD_IntakeOn;
import frc.robot.commands.CMD_Place1stLevel;
import frc.robot.commands.CMD_Place2ndConeLevel;
import frc.robot.commands.CMD_Place3rdConeLevel;
import frc.robot.commands.CMD_Place3rdCubeLevel;
import frc.robot.commands.CMD_SetInitalOdometry;
import frc.robot.commands.CMD_Stow;
import frc.robot.commands.CMD_selectIntakeCommandKey;
import frc.robot.commands.CMD_setDropLevel;
import frc.robot.commands.CMD_setInitialOdometeryHolonomic;
import frc.robot.commands.CMD_setIntakeState;
import frc.robot.commands.CMD_setPickUpMode;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_PP2BalanceSpeedBumpBlue extends SequentialCommandGroup {
  /** Creates a new AUTO_PPFullLinkDivider. */
  AUTO_Trajectories m_trajectories;
  public AUTO_PP2BalanceSpeedBumpBlue(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain, SUB_Elbow p_elbow, SUB_Elevator p_elevator,
  SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables, SUB_Intake p_intake, CommandXboxController p_controller){
    m_trajectories = p_trajectories;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.LinkRunTrajectoryRed1).withTimeout(3),
      new CMD_setIntakeState(p_variables, GlobalConstants.kConeMode).withTimeout(3),
      new CMD_setDropLevel(p_variables, GlobalConstants.kElevator3rdLevel).withTimeout(3),
      new CMD_setPickUpMode(p_variables, GlobalConstants.kPickBackGroundMode).withTimeout(3),
      new CMD_IntakeHold(p_intake, p_variables).withTimeout(3),
      new CMD_selectIntakeCommandKey(p_intake, p_variables).withTimeout(3),
      new CMD_Place3rdConeLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables).withTimeout(3),
      new CMD_IntakeDropAuto(p_intake, p_variables),
      new WaitCommand(.2),
      new CMD_setIntakeState(p_variables, GlobalConstants.kCubeMode),
      new CMD_Stow(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables).withTimeout(3),
      new CMD_setInitialOdometeryHolonomic(p_drivetrain, m_trajectories.CubeRunBlueSpeedBump),
      new ParallelCommandGroup(
        m_trajectories.followTrajectoryCommand(m_trajectories.CubeRunBlueSpeedBump),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new ParallelCommandGroup(
            new CMD_GroundCubeIntake(p_intake, p_elbow, p_elevator, p_finiteStateMachine).withTimeout(3),
            new CMD_IntakeOn(p_intake, p_variables)    
          )
        )
      ),
      new ParallelCommandGroup(
        m_trajectories.followTrajectoryCommand(m_trajectories.CubePlaceBlueSpeedBump), 
        new CMD_Place1stLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables).withTimeout(3)
      ),
      new CMD_Place3rdConeLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables).withTimeout(3),
      new CMD_IntakeDropAuto(p_intake, p_variables),
      new WaitCommand(.2),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new CMD_CheckOnCharge(p_drivetrain).withTimeout(3),
              new WaitCommand(AutoConstants.AutoBalanceTimer)  
            ),
            m_trajectories.followTrajectoryCommand(m_trajectories.ParkBlueSpeedBump)
            )
        ),
        new CMD_Stow(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables).withTimeout(3)
      ),
      new WaitCommand(1),
      new CMD_AdjustBalanceInside(p_drivetrain)
    );
  }
}
