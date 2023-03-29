// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_LimelightSetPipeline extends CommandBase {
  SUB_Limelight m_limelight;
  int m_pipeline;
  public CMD_LimelightSetPipeline(SUB_Limelight p_limelight, int p_pipeline) {
    m_limelight = p_limelight;
    m_pipeline = p_pipeline;
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(m_pipeline);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
