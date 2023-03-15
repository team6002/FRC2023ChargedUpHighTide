// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_PrepIntakeGroundBack extends SequentialCommandGroup {
  /** Creates a new CMD_PrepIntakeGroundBack. */
  public CMD_PrepIntakeGroundBack(SUB_Elbow p_elbow, SUB_Elevator p_elevator,
  SUB_FiniteStateMachine p_finiteStatemachine,SUB_Intake p_intake, GlobalVariables m_Variables) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_setState(p_finiteStatemachine, RobotState.PREPINTAKE),
      new CMD_IntakeOff(p_intake),
      new ParallelCommandGroup(
      new CMD_ElevatorSetPosition(p_elevator, ElevatorConstants.kElevatorHome)
      ),
      new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowStowBackwards)
    );
  }
}
