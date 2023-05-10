// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants;

public class GlobalVariables extends SubsystemBase {
  /** Creates a new GlobalVariables. */
  private boolean m_intakeState = true;// true for cone mode, false for cube mode
  private boolean m_hasItem = false;//false is no item true is has item
  private boolean m_AutoDrop = true;// automatically drops it once its good or not
  private boolean m_dropGood = false;
  private int m_dropLevel = 2;// 1 is ground, 2 is second level, 3 is third level
  private int m_stowLocation = 1;// 0 is ground, 1 is shelf
  private int m_gridposition = -1;
  private int m_pickMode = -1;// 0 is groundBack, 1 is groundForwards, 2 shelfBack, 3 shelfForwards
  private int m_intakeCommandKey = -1;
  private int m_stage = 0;// 0 is intake, 2 is drop, 3 is dropped
  private int m_autoKey = 0;
  private int m_extendKey = -1;
  private int m_PickAlignPosition = -1;
  private int m_PlaceAlignPosition = -1;
  private int m_RetroflectiveAlignPosition = 0;// 0 is middle, 1 is left, 2 is right 
  

  private Constants.AutoAlignConstants.AlignPosition m_AlignPosition;

  public GlobalVariables() {}

  public void setIntakeState(boolean p_state){
    m_intakeState = p_state;
  }

  public boolean getIntakeState(){
    return m_intakeState;
  }

  public void setHasItem(boolean p_item){
    m_hasItem = p_item;
  }

  public boolean getHasItem(){
    return m_hasItem;
  }

  public void setPickUpAlignPosition(int p_alignPosition){
    m_PickAlignPosition = p_alignPosition;
  }

  public int getPickUpAlignPosition(){
    return m_PickAlignPosition;
  }

  public void setPlaceUpAlignPosition(int p_alignPosition){
    m_PlaceAlignPosition = p_alignPosition;
  }

  public int getPlaceAlignPosition(){
    return m_PlaceAlignPosition;
  }

  public void setAlignPosition(Constants.AutoAlignConstants.AlignPosition p_alignPosition){
    m_AlignPosition = p_alignPosition;
  }

  public Constants.AutoAlignConstants.AlignPosition getAlignPosition(){
    return m_AlignPosition;
  }

  public void setDropLevel(int p_level){
    m_dropLevel = p_level;
  }
  public int getDropLevel(){
    return m_dropLevel;
  }

  public void setStowLocation(int p_Location){
    m_stowLocation = p_Location;
  }

  public int getStowLocaton(){
    return m_stowLocation;
  }

  public void setGrid(int p_position){
    m_gridposition = p_position;
  } 

  public int getGrid(){
    return m_gridposition;
  }

  public void setPickMode(int p_mode){
    m_pickMode = p_mode;
  }

  public int getPickMode(){
    return m_pickMode;
  }

  public int getIntakeCommandKey(){
    return m_intakeCommandKey;
  }

  public void setIntakeCommandKey(int p_command){
    m_intakeCommandKey = p_command;
  }

  public int getStage(){
    return m_stage;
  }

  public void setStage(int p_state){
    m_stage = p_state;
  }

  public int getAutoKey(){
    return m_autoKey;
  }

  public void setAutoKey(int p_autoKey){
    m_autoKey = p_autoKey;
  }

  public int getExtendKey(){
    return m_extendKey;
  }

  public void setExtendKey(int p_extendKey){
    m_extendKey = p_extendKey;
  }

  public int getRetoflectiveAlignPosition(){
    return m_RetroflectiveAlignPosition;
  }

  public void setRetroflectiveAlignPosition(int p_RetroflectiveAlignPosition){
    m_RetroflectiveAlignPosition = p_RetroflectiveAlignPosition;
  }

  public boolean getAutoDrop(){
    return m_AutoDrop;
  }

  public void setAutoDrop(boolean p_autodrop){
    m_AutoDrop = p_autodrop;
  }
  
  public boolean getDropGood(){
    return m_dropGood;
  }

  public void setDropGood(boolean p_dropGood){
    m_dropGood = p_dropGood;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeCommandKey", m_intakeCommandKey);
    SmartDashboard.putBoolean("Element?", m_hasItem);
    SmartDashboard.putBoolean("IntakeMode", m_intakeState);
    SmartDashboard.putNumber("DropLevel", m_dropLevel);
    SmartDashboard.putNumber("StowLocation", m_stowLocation);
    SmartDashboard.putNumber("PickMode", m_pickMode);
  }
}
