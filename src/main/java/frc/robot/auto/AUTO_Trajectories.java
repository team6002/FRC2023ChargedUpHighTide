package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SUB_Drivetrain;
public class AUTO_Trajectories {

    private SUB_Drivetrain m_drivetrain;

    //divider trajectories

    //blue
    public PathPlannerTrajectory ConeRunBlueDivider = PathPlanner.loadPath("ConeRunBlueDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    public PathPlannerTrajectory ConePlaceBlueDivider = PathPlanner.loadPath("ConePlaceBlueDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
    public PathPlannerTrajectory CubeRunBlueDivider = PathPlanner.loadPath("CubeRunBlueDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
    public PathPlannerTrajectory CubePlaceBlueDivider = PathPlanner.loadPath("CubePlaceBlueDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
    public PathPlannerTrajectory ParkBlueDivider = PathPlanner.loadPath("ParkBlueDivider",
        new PathConstraints(AutoConstants.kChargeStationSpeed,
        AutoConstants.kChargeStationAcceleration));

    //red
    public PathPlannerTrajectory ConeRunRedDivider = PathPlanner.loadPath("ConeRunRedDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    public PathPlannerTrajectory ConePlaceRedDivider = PathPlanner.loadPath("ConePlaceRedDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    public PathPlannerTrajectory CubeRunRedDivider = PathPlanner.loadPath("CubeRunRedDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    public PathPlannerTrajectory CubePlaceRedDivider = PathPlanner.loadPath("CubePlaceRedDivider",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    public PathPlannerTrajectory ParkRedDivider = PathPlanner.loadPath("ParkRedDivider",
        new PathConstraints(AutoConstants.kChargeStationSpeed,
        AutoConstants.kChargeStationAcceleration));
    //charge station

    //blue

    public PathPlannerTrajectory CubeRunBlueChargeStation = PathPlanner.loadPath("CubeRunBlueChargeStation",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
    public PathPlannerTrajectory ParkBlueChargeStation = PathPlanner.loadPath("ParkBlueChargeStation",
        new PathConstraints(AutoConstants.kChargeStationSpeed,
        AutoConstants.kChargeStationAcceleration));

    //red
    
    public PathPlannerTrajectory CubeRunRedChargeStation = PathPlanner.loadPath("CubeRunRedChargeStation",
        new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    public PathPlannerTrajectory ParkRedChargeStation = PathPlanner.loadPath("ParkRedChargeStation",
        new PathConstraints(AutoConstants.kChargeStationSpeed,
        AutoConstants.kChargeStationAcceleration));

    //test

    public PathPlannerTrajectory test = PathPlanner.loadPath("test", new PathConstraints(1, 1));    
    

    public AUTO_Trajectories(SUB_Drivetrain drivetrain){
        m_drivetrain = drivetrain;
    }
  
    public Command driveTrajectory(Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
            0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
            m_drivetrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0), // Position controllers
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drivetrain::setModuleStates,
            m_drivetrain);
        
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0.0 ,0.0 ,0.0, true, false));
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_drivetrain.resetOdometry(traj.getInitialHolonomicPose())),
            new PPSwerveControllerCommand(
                traj,//trajectory
                m_drivetrain::getPose, // Pose supplier
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                new PIDController(AutoConstants.kPPPXController, 0, 0), //X positon controller
                new PIDController(AutoConstants.kPPPYController, 0, 0), //Y position controller
                new PIDController(AutoConstants.kPPPThetaController, 0, 0), //turn theta controller
                m_drivetrain::setModuleStates,
                false, //use alliance color
                m_drivetrain)//subsystems requirements
        );
        }
}