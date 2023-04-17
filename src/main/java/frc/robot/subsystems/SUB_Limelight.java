// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.LimeLightConstants;

public class SUB_Limelight extends SubsystemBase {
  
  private final int m_aprilTagPipelineId = LimeLightConstants.kAprilTagPipelineId;
  private final int m_cone2ndPipelineId = LimeLightConstants.kCone2ndPipelineId;
  private final int m_cone3rdPipelineId = LimeLightConstants.kCone3rdPipelineId;
  private int m_currentPipelineId;

  public SUB_Limelight() {
    /*
    * 0 - Standard - Side-by-side streams if a webcam is attached to Limelight
    * 1 - PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
    * 2 - PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
    */
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(CameraConstants.kLimelightIndex);

    setPipeline(m_cone2ndPipelineId);
  }

  private double[] dv = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public double[] botpose;

  @Override
  public void periodic() {
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(dv);

    telemetry();
  }

  public void telemetry(){
    // SmartDashboard.putBoolean("DO YOU SEE ANYTHING ", hasTarget());

    if (hasTarget()) {
      SmartDashboard.putNumber("target ID", getTargetID());
      // SmartDashboard.putNumber("target x", getTargetX());
      // SmartDashboard.putNumber("target Y", getTargetY());
      // SmartDashboard.putNumber("target Z", getTargetZ());
      // SmartDashboard.putNumber("target pitch", getTargetPitch());
      // SmartDashboard.putNumber("target yaw", getTargetYaw());
      // SmartDashboard.putNumberArray("botpose", botpose);
    }
  }

  public boolean hasTarget(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
      return true;
    }
    else{
      return false;
    }
  }

  public double getTargetID(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  }

  public double getTargetTx() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    return tx;
  }

  
  public double getTargetTy() {
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    return ty;
  }

  public int readGrid(){
    double m_id = getTargetID();
    if (m_id == 3 ||m_id == 8){
      return GlobalConstants.kLeftGrid;
    }else if (m_id == 2 ||m_id == 7){
      return GlobalConstants.kMiddleGrid;
    }else if (m_id == 1 ||m_id == 6){
      return GlobalConstants.kRightGrid;
    } else
    return -1;
  }

  public Pose2d getRobotPoseInTargetSpace() {
    // System.out.println("botpose[4]= " + botpose[4]);
    // System.out.println(Rotation2d.fromDegrees(botpose[4]));
    // Pose2d pose = new Pose2d(botpose[2], -botpose[0], Rotation2d.fromDegrees(0));
    Pose2d pose = new Pose2d(-botpose[2], botpose[0], Rotation2d.fromDegrees(-botpose[4]));

    return pose;
  }

  public double getTargetX(){
    return -botpose[2];
  }

  public double getTargetY(){
    return botpose[0];
  }

  public double getTargetZ(){
    return Units.metersToInches(botpose[1]);
  }

  public double getTargetPitch(){
    return botpose[3];
  }

  public double getTargetYaw(){
    return -botpose[4];
  }

  public void useMainCamera(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(CameraConstants.kLimelightIndex);
  }

  public void useSecondaryCamera(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(CameraConstants.kDriveCamIndex);
  }


  public void setPipeline(double m_wantedPipeline) {
    if (m_currentPipelineId == m_wantedPipeline){
      return;
    }else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(m_wantedPipeline);
    }
  }
}