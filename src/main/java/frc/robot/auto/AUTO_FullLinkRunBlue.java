// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class AUTO_FullLinkRunBlue extends SequentialCommandGroup {
  public AUTO_FullLinkRunBlue(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain, SUB_Elbow p_elbow, SUB_Elevator p_elevator,
   SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables, SUB_Intake p_intake, CommandXboxController p_controller) {
    addCommands(
      // new ParallelRaceGroup(
        new SequentialCommandGroup(
          new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.LinkRunTrajectoryBlue1),
          new CMD_setIntakeState(p_variables, GlobalConstants.kConeMode),
          new CMD_setDropLevel(p_variables, GlobalConstants.kElevator3rdLevel),
          new CMD_setPickUpMode(p_variables, GlobalConstants.kPickBackGroundMode),
          new CMD_IntakeHold(p_intake, p_variables),
          new CMD_selectIntakeCommandKey(p_intake, p_variables),
          new CMD_Place3rdCubeLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables),
          new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowDrop),
          new CMD_IntakeDrop(p_intake, p_variables),
          new WaitCommand(.3),
          new CMD_IntakeStop(p_intake),
          new CMD_setDropLevel(p_variables, GlobalConstants.kElevator3rdLevel),
          new CMD_setIntakeState(p_variables, GlobalConstants.kCubeMode),
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new WaitCommand(.5),
              p_trajectories.driveTrajectory(p_trajectories.LinkRunTrajectoryBlue1)   
            ),
            new CMD_GroundCubeIntake(p_intake, p_elbow, p_elevator, p_finiteStateMachine),
            new CMD_IntakeOn(p_intake, p_variables)
          ),
          new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.LinkPlaceTrajectoryBlue1),
          new ParallelDeadlineGroup(
            p_trajectories.driveTrajectory(p_trajectories.LinkPlaceTrajectoryBlue1),
            new SequentialCommandGroup(
              new WaitCommand(.5),
              new CMD_GroundHold(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables) 
            )
          ),
          new CMD_Place3rdCubeLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables),
          new CMD_IntakeDrop(p_intake, p_variables),
          new WaitCommand(.3)
        ),
        new CMD_IntakeStop(p_intake),
        new CMD_setDropLevel(p_variables, GlobalConstants.kElevator1stLevel),
        new CMD_setIntakeState(p_variables, GlobalConstants.kCubeMode),
        new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.LinkRunTrajectoryBlue2),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new WaitCommand(.5),
            p_trajectories.driveTrajectory(p_trajectories.LinkRunTrajectoryBlue2)
          ),
          new SequentialCommandGroup(
            // new CMD_IntakeOn(p_intake, p_variables),
            // new SequentialCommandGroup(
            //   new WaitCommand(2)
            new CMD_Stow(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables)
              // new CMD_GroundCubeIntake(p_intake, p_elbow, p_elevator, p_finiteStateMachine)    
            // )
          )
        ),
        new ParallelCommandGroup(
          new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.LinkPlaceTrajectoryBlue2),
          p_trajectories.driveTrajectory(p_trajectories.LinkPlaceTrajectoryBlue2),
          new SequentialCommandGroup(
            new WaitCommand(.2),
            new CMD_GroundHold(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables)
          )  
        ),
        new CMD_Place1stLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables),
        new CMD_IntakeDrop(p_intake, p_variables),
        new WaitCommand(0.2),
        new CMD_IntakeStop(p_intake),
        new CMD_Stow(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables)
    );
  }
}