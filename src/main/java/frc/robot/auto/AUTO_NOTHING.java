// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AUTO_NOTHING extends CommandBase {
  /** Creates a new AUTO_NOTHING. */
  public AUTO_NOTHING() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //It does NOTHING!
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //It still does NOTHING!!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SURPRISE it does NOTHING!!!
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
