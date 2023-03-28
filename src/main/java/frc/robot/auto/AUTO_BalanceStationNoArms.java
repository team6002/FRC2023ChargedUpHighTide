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
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Cogfnsider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_BalanceStationNoArms extends SequentialCommandGroup {
  /** Creates a new AUTO_BalanceStation. */
  public AUTO_BalanceStationNoArms(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain,
    SUB_Elbow p_elbow, SUB_Elevator p_elevator, SUB_Intake p_intake, 
    SUB_FiniteStateMachine p_finiteStateMachine,GlobalVariables p_variables,
    CommandXboxController p_controller) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AUTO_DriveOverChargingStation(p_trajectories, p_drivetrain),
      new CMD_SpinInPlace(p_drivetrain, 180).withTimeout(4),
      new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.BackOnChargeStationTrajectory),
      new ParallelDeadlineGroup(
          new SequentialCommandGroup(    
            new CMD_CheckOnCharge(p_drivetrain).withTimeout(3)
            ,new WaitCommand(1.1)//1.03 was St Joe // 1.73 is td
          ),
          new AUTO_DriveBackOnChargeStation(p_trajectories, p_drivetrain)
      ),
      new CMD_DriveStop(p_drivetrain).withTimeout(3),
      new WaitCommand(1),
      new CMD_AdjustBalanceOutside(p_drivetrain).withTimeout(3),
      new CMD_ResetGyro(p_drivetrain).withTimeout(3)
    );
  }
}
