// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class CMD_GroundHold extends SequentialCommandGroup {
  /** Creates a new GroundIntake. */
  public CMD_GroundHold(SUB_Intake p_intake, SUB_Elbow p_elbow, SUB_Elevator p_elevator, SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_setState(p_finiteStateMachine, RobotState.STOW),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new CMD_IntakeReadjust(p_intake, p_variables).withTimeout(.25),
          new CMD_IntakeHold(p_intake, p_variables)   
        ),
        new CMD_ElevatorSetPosition(p_elevator, ElevatorConstants.kElevatorGround),
        new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowUp)
      ),
      new WaitCommand(.1),
      new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowStow)
      
    );
  }
}
