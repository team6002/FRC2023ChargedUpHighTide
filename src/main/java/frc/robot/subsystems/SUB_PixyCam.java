// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pixy2api.Pixy2;
import frc.robot.pixy2api.Pixy2CCC;
import frc.robot.pixy2api.Pixy2CCC.Block;
import frc.robot.pixy2api.links.Link;
import frc.robot.pixy2api.links.SPILink;

public class SUB_PixyCam extends SubsystemBase {
  /** Creates a new SUB_PixyCam. */
  Pixy2 m_pixyCam;
  int blockCount;
  public SUB_PixyCam() {
    // m_pixyCam = Pixy2.createInstance(m_SPIlink);
    m_pixyCam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    m_pixyCam.init();
    
  }

  public double getSomething(){
    int blockCount = m_pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);

    ArrayList<Block> blocks = m_pixyCam.getCCC().getBlockCache();

    if (blocks.size() == 0){
      return -1;
    }else {
      return blocks.get(0).getX();
    }

  }
    // return m_pixyCam.getFPS();
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("PiXy", getSomething());
    // This method will be called once per scheduler run
  }
}
