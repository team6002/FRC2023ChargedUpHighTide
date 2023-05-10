// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.commands.CMD_IntakeDropAuto;
import frc.robot.commands.CMD_Place3rdConeLevel;
import frc.robot.commands.CMD_Stow;
import frc.robot.commands.CMD_selectIntakeCommandKey;
import frc.robot.commands.CMD_setDropLevel;
import frc.robot.commands.CMD_setIntakeState;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_3WireBridgeCamera extends SequentialCommandGroup {
  public AUTO_3WireBridgeCamera(GlobalVariables p_variables, SUB_Intake p_intake, SUB_Elbow p_elbow, SUB_Elevator p_elevator,
    SUB_FiniteStateMachine p_finiteStateMachine) {
    addCommands(
      new CMD_setDropLevel(p_variables, GlobalConstants.kElevator1stLevel),
      new CMD_setIntakeState(p_variables, GlobalConstants.kConeMode),
      new CMD_selectIntakeCommandKey(p_intake, p_variables),
      new CMD_Place3rdConeLevel(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables),
      new CMD_IntakeDropAuto(p_intake, p_variables),
      new CMD_Stow(p_intake, p_elbow, p_elevator, p_finiteStateMachine, p_variables)
    );
  }
}
