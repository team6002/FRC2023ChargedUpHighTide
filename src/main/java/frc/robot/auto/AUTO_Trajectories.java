    package frc.robot.auto;
    import com.pathplanner.lib.PathConstraints;
    import com.pathplanner.lib.PathPlanner;
    import com.pathplanner.lib.PathPlannerTrajectory;
    import com.pathplanner.lib.commands.PPSwerveControllerCommand;

    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    import java.util.List;

    import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.math.controller.ProfiledPIDController;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.math.trajectory.Trajectory;
    import edu.wpi.first.math.trajectory.TrajectoryConfig;
    import edu.wpi.first.math.trajectory.TrajectoryGenerator;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
    import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
    import frc.robot.Constants.AutoConstants;
    import frc.robot.Constants.DriveConstants;
    import frc.robot.subsystems.SUB_Drivetrain;
    /** Add your docs here. */

    public class AUTO_Trajectories {

    public Trajectory OverChargeStationTrajectory;
    public Trajectory BackOnChargeStationTrajectory;
    public Trajectory OverChargeStationTrajectory2;

    public Trajectory LinkRunTrajectoryRed1;
    public Trajectory LinkPlaceTrajectoryRed1;
    public Trajectory LinkPlaceTrajectoryRed2;
    public Trajectory LinkRunTrajectoryRed2;
    public Trajectory LinkBalanceTrajectoryRed;

    public Trajectory LinkRunTrajectoryBlue1;
    public Trajectory LinkPlaceTrajectoryBlue1;
    public Trajectory LinkPlaceTrajectoryBlue2;
    public Trajectory LinkRunTrajectoryBlue2;
    public Trajectory LinkBalanceTrajectoryBlue;

    Trajectory FirstRedBall = new Trajectory();
    private SUB_Drivetrain m_drivetrain;

    public AUTO_Trajectories(SUB_Drivetrain drivetrain){
        m_drivetrain = drivetrain;

        TrajectoryConfig ChargeStationConfig =
            new TrajectoryConfig(
                AutoConstants.kChargeStationSpeed,
                AutoConstants.kChargeStationAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        TrajectoryConfig ChargeStationConfigReversed =
            new TrajectoryConfig(
                AutoConstants.kChargeStationSpeed,
                AutoConstants.kChargeStationAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);
        
        TrajectoryConfig LinkChargeStationConfigReversed =
        new TrajectoryConfig(
            AutoConstants.k2LinkChargeStationSpeed,
            AutoConstants.k2LinkChargeStationAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);

        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        TrajectoryConfig configReversed =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);

        
        TrajectoryConfig configHalfSpeed =
        new TrajectoryConfig(
            AutoConstants.kHalfSpeed,
            AutoConstants.kHalfAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        TrajectoryConfig configHalfSpeedReversed =
        new TrajectoryConfig(
            AutoConstants.kHalfSpeed,
            AutoConstants.kHalfAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);    
                    
        //Rotation2d uses RADIANS NOT DEGREES!
        //Use Rotation2d.fromDegrees(desiredDegree) instead

        // Balance Station trajectories
        OverChargeStationTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-120), 0, new Rotation2d(0)),
            ChargeStationConfigReversed);

        OverChargeStationTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-120), 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-220), Units.inchesToMeters(0), new Rotation2d(0)),
            configReversed);

        BackOnChargeStationTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-220), 0, new Rotation2d(Units.degreesToRadians(180))),
            List.of(),
            new Pose2d(Units.inchesToMeters(-70), 0, new Rotation2d(Units.degreesToRadians(180))),
            ChargeStationConfigReversed); //-68 was twisted devil -86 was strykforce
        
        
        // Full Link Run Red side trajectories (goes Negative)
        LinkRunTrajectoryRed1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(-18), new Rotation2d(0)),
            configReversed);
        
        LinkPlaceTrajectoryRed1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(-18), new Rotation2d(0)),
                List.of(),
                new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(-18), new Rotation2d(Units.degreesToRadians(-15))),
                config);

        // this is the original one where it lines up with the cube box
        // LinkPlaceTrajectoryRed1 = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(-18), new Rotation2d(0)),
        //     List.of(),
        //     new Pose2d(Units.inchesToMeters(-0), Units.inchesToMeters(-25), new Rotation2d(0)),
        //     config);

        LinkBalanceTrajectoryRed = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(-18), new Rotation2d(Units.degreesToRadians(-15))),
            List.of(new Translation2d(Units.inchesToMeters(-30), Units.inchesToMeters(-70))),
            new Pose2d(Units.inchesToMeters(-140), Units.inchesToMeters(-80), new Rotation2d(Units.degreesToRadians(0))),
            LinkChargeStationConfigReversed);


        LinkRunTrajectoryRed2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(-18), new Rotation2d(Units.degreesToRadians(-15))),
            List.of(new Translation2d(Units.inchesToMeters(-110), Units.inchesToMeters(-18))),
            new Pose2d(Units.inchesToMeters(-207), Units.inchesToMeters(-65), new Rotation2d(Units.degreesToRadians(45))),
            configReversed);

        LinkPlaceTrajectoryRed2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-207), Units.inchesToMeters(-65), new Rotation2d(Units.degreesToRadians(45))),
            List.of(new Translation2d(Units.inchesToMeters(-130), Units.inchesToMeters(-18)),
            new Translation2d(Units.inchesToMeters(-110), Units.inchesToMeters(-18))
            ),
            new Pose2d(Units.inchesToMeters(-0), Units.inchesToMeters(-18), new Rotation2d(Units.degreesToRadians(0))),
            // new Pose2d(Units.inchesToMeters(-0), Units.inchesToMeters(-30), new Rotation2d(Units.degreesToRadians(0))),
            config);
        
        // Full Link Run Blue side trajectories (goes Positive)

        LinkRunTrajectoryBlue1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(18), new Rotation2d(0)),
            configReversed);

        LinkPlaceTrajectoryBlue1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(18), new Rotation2d(Units.degreesToRadians(-15))),
                List.of(),
                new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(18), new Rotation2d(Units.degreesToRadians(15))),
                config);

        // this is the original one where it lines up with the cube box
        // LinkPlaceTrajectoryBlue1 = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(-18), new Rotation2d(0)),
        //     List.of(),
        //     new Pose2d(Units.inchesToMeters(-0), Units.inchesToMeters(-25), new Rotation2d(0)),
        //     config);

        LinkBalanceTrajectoryBlue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(18), new Rotation2d(Units.degreesToRadians(15))),
            List.of(new Translation2d(Units.inchesToMeters(-30), Units.inchesToMeters(70))),
            new Pose2d(Units.inchesToMeters(-140), Units.inchesToMeters(80), new Rotation2d(Units.degreesToRadians(0))),
            LinkChargeStationConfigReversed);

        LinkRunTrajectoryBlue2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(18), new Rotation2d(-Units.degreesToRadians(-45))),
            List.of(new Translation2d(Units.inchesToMeters(-110), Units.inchesToMeters(18))),
            new Pose2d(Units.inchesToMeters(-207), Units.inchesToMeters(65), new Rotation2d(Units.degreesToRadians(-45))),
            configReversed);

        LinkPlaceTrajectoryBlue2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-207), Units.inchesToMeters(65), new Rotation2d(Units.degreesToRadians(-45))),
            List.of(new Translation2d(Units.inchesToMeters(-130), Units.inchesToMeters(18)),
            new Translation2d(Units.inchesToMeters(-110), Units.inchesToMeters(18))
            ),
            new Pose2d(Units.inchesToMeters(-0), Units.inchesToMeters(18), new Rotation2d(Units.degreesToRadians(0))),
            // new Pose2d(Units.inchesToMeters(-0), Units.inchesToMeters(-30), new Rotation2d(Units.degreesToRadians(0))),
            config);
            
        }
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

        public PathPlannerTrajectory ConeParkBlueDivider = PathPlanner.loadPath("ConeParkBlueDivider",
        new PathConstraints(AutoConstants.kChargeStationSpeed,
        AutoConstants.kChargeStationAcceleration));

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
