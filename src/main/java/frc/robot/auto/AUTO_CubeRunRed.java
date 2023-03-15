// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class AUTO_CubeRunRed extends SequentialCommandGroup {
  public AUTO_CubeRunRed(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain, SUB_Elbow p_elbow, SUB_Elevator p_elevator,
  SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables, SUB_Intake p_intake, CommandXboxController p_controller) {
    addCommands(
      // new ParallelRaceGroup(
        new SequentialCommandGroup(
          new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.CubeRunTrajectoryRed1),
          new CMD_setIntakeMode(p_variables, GlobalConstants.kConeMode),
          new CMD_setDropLevel(p_variables, GlobalConstants.kElevator3rdLevel),
          new CMD_setPickUpMode(p_variables, GlobalConstants.kPickBackGroundMode),
          new CMD_PlaceForwardsCone(p_elevator, p_intake, p_elbow, p_finiteStateMachine, p_variables),
          new CMD_IntakeDrop(p_intake, p_variables),
          new WaitCommand(.3),
          new CMD_Stow(p_elevator, p_intake, p_elbow, p_finiteStateMachine)
    ));
  }
}