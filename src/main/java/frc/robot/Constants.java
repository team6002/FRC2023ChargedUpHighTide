// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 14;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 13;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 3;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = .00008;// 0.01;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.167; //.203;
    public static final double kDrivingS = 0.1;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1.5; //.7;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kChargeStationSpeed = 1.12;
    public static final double kChargeStationAcceleration = 1.12;
    public static final double k2LinkChargeStationSpeed = 1.5;// theres not enough time for the speed to build up for 2 link balance
    public static final double k2LinkChargeStationAcceleration = 1.5;
    public static final double kHalfSpeed = 2;
    public static final double kHalfAcceleration = 2;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 2;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    public static final int kBalanceStationKey = 0;
    public static final int kCubeRunKey = 1;

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants{
    public static final int kIntakeMotorCanID = 12;
    //intake powers
    public static final double kIntakeForwardPower = .6;// for cones
    public static final double kIntakeBackwardPower = -.6;// for cubes
    public static final double kIntakeConeDownPower = .1;
    public static final double kIntakeHoldPower = .07;
    public static final double kIntakeExtraHoldPower = .5;
    public static final double kIntakeDropCone = -.6;
    public static final double kIntakeDropCube = 0.3;// the cubes are lighter than cones
    public static final double kIntakeDropGroundCube = 0.1;// the cubes are lighter than cones
    public static final double kIntakeShoot = .4;
    //intake currents
    public static final int kIntakeCurrent = 50;
    public static final int kReholdCurrent = 10;
    public static final int kHoldCurrent = 5;

    public static final int kIntakeDetectedtimer = 5;  

    public static final double kIntakeConeDetectedCurrent = 30;

  }

  public static final class ElevatorConstants{
    public static final int kElevatorMotorCanID = 17;
    public static final double kElevatorP = 0.65;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0.;
    public static final double kElevatorF = 0.0;
    public static final double kElevatorOffset = -.487;

    public static final double kElevatorMax = 54;
    public static final double kElevatorShelfCone = 41;
    public static final double kElevatorShelfCube = 39;
    public static final double kElevatorFirstConeLevel = 9;// for placeing forwards
    public static final double kElevatorSecondConeLevel = 31; 
    public static final double kElevatorThirdConeLevel = 52;
    public static final double kElevatorFirstCubeLevel = 9;// for placeing forwards
    public static final double kElevatorSecondCubeLevel = 31;  
    public static final double kElevatorThirdCubeLevel = 50;
    public static final double kElevatorHome = 0;
    public static final double kElevatorStow = 0;
    public static final double kElevatorGround = 0;
    public static final double kElevatorPrep = 19;//Position for safely moving elbow to score
    public static final double kElevatorSafety = 12;//maximum value to check if moving elbow endagers robot
    public static final double kElevatorTolerance = 2;
    public static final double kElevatorShelfBack = 18;
    public static final double kElevatorShelfBackAway = 24;// moves the elevator up to avoid hitting the wall

    public static final double kElevatorHomeCurrent = 10;

    public static final double kElevatorMaxVelocity = 120;
    public static final double kElevatorMaxAcceleration = 160;
    public static final double kElevatorHomeAcceleration = 3;
    public static final double kElevatorHomeVelocity = 3;
    
  }

  public static final class ElbowConstants{
    public static final int kElbowMotorCanID = 11;
    public static final double kElbowP = 0.04;//.008;
    public static final double kElbowI = 0;
    public static final double kElbowD = 0.0049261;
    public static final double kElbowF = 0.00;
    public static final double kElbowS = 0.0062947;
    public static final double kElbowV = 0.03199;
    public static final double kElbowMaxVelocity = 600;
    public static final double kElbowMaxAcceleration = 600;
    public static final double kElbowMinOutput = -1;
    public static final double kElbowMaxOutput = 1;

    // public static final double kElbowLift = 149;// use to get elbow off elevator, it don't like 200 for some reason
    public static final double kElbowLifted = 125;// when the elbow is not resting on elevator
    public static final double kElbowStow = 149;
    public static final double kElbowUp = 135;// about 90 degrees up
    public static final double kElbowForwards = 149;// stright forwards
    public static final double kElbowShelf = 133;// stright forwards
    public static final double kElbowPrepareDrop = 140;// Not quite all the way forwards, allows cones to clear the pole
    public static final double kElbowDrop = 145;// Not quite all the way forwards, allows cones to clear the pole
    public static final double kElbowBackwards = 23;// straight back
    // public static final double kElbowThrow = 60;//for throwing the cubes for MAXIMUM distance
    public static final double kElbowGroundConeUpright = -6;// when the cone stands up straight
    // public static final double kElbowGroundConeDown = 21;// when the cone are knocked over
    public static final double kElbowGroundCube = -6;// when picking up cube
    //depricated
    // public static final double kElbowShootCube = 210;//for shooting cubes
    // public static final double kElbowShelfBack = 95;
    // public static final double kElbowShelfBackPrep = 85;
  }
  

  public static final class LimeLightConstants{
    public static final double[] kTarget1Constants = {1, 1, 0};
    public static final double[] kTarget2Constants = {2, 2, 0};
    public static final double[] kTarget3Constants = {3, 3, 0};
    public static final double[] kTarget4Constants = {4, 4, 0};
    public static final double[] kTarget5Constants = {5, 5, 180};
    public static final double[] kTarget6Constants = {6, 6, 180};
    public static final double[] kTarget7Constants = {7, 7, 180};
    public static final double[] kTarget8Constants = {8, 8, 180};
  }

  public static final class AutoAlignConstants{
    /* X and Y drive constraints. Output ranges [-1, 1] */
    public static final double kXTolerance = 0.05;
    public static final double kYTolerance = 0.05;

    public static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(1.25, 1.25);
    public static final double driveKp = 1.5;
    public static final double driveKi = 0.;
    public static final double driveKd = 0.;

    /* Turn constraints. Output ranges [-1, 1] */
    public static final double kTurnTolerance = 5;
    public static final TrapezoidProfile.Constraints turnConstraints = new TrapezoidProfile.Constraints(1, 1);
    public static final double turnKp = 0.01;
    public static final double turnKi = 0.;
    public static final double turnKd = 0.;

    /* Absolute joystick threshold for driver abort */
    public static final double kAbortThreshold = 0.2;

    public static enum AlignPosition {
      LEFTSCORE, //april tagids:3 red, 8 blue 
      MIDDLESCORE, //april tagids:2 red, 7 blue
      RIGHTSCORE, //april tagids:1 red, 6 blue
      
      LEFTSHELF,
      MIDDLESHELF,// this isn't a pick up location but the middle in front of the apriltag
      RIGHTSHELF,
    };

    public static final EnumMap<AlignPosition, Pose2d> goalPose = new EnumMap<>(Map.of(
      AlignPosition.LEFTSCORE, new Pose2d(-0.85, 0.5, new Rotation2d()),
      AlignPosition.MIDDLESCORE, new Pose2d(-0.85, 0.0, new Rotation2d()),
      AlignPosition.RIGHTSCORE, new Pose2d(-0.85, -0.5, new Rotation2d()),
      AlignPosition.LEFTSHELF, new Pose2d(-1.3, .3, new Rotation2d()),
      AlignPosition.MIDDLESHELF, new Pose2d(-1.3, 0, new Rotation2d()),
      AlignPosition.RIGHTSHELF, new Pose2d(-1.3, -.3, new Rotation2d())
    ));
  }

  public static final class GlobalConstants{
    public static final int kUnknownPickMode = -1;
    public static final int kPickBackGroundMode = 0;
    public static final int kPickForwardsShelfMode = 1;
    public static final int kPickConeDownMode = 2;
    
    public static final int kStowGroundMode = 0;
    public static final int kStowShelfMode = 1;

    public static final int kElevator1stLevel = 1;
    public static final int kElevator2ndLevel = 2;
    public static final int kElevator3rdLevel = 3;

    public static final boolean kConeMode = true;
    public static final boolean kCubeMode = false;

    public static final int kUnknownIntakeKey = -1;
    public static final int kGroundBackCube = 0;
    public static final int kGroundBackConeUpright = 1;
    public static final int kGroundBackConeDown = 2;
    public static final int kShelfForwardsCube = 3;
    public static final int kShelfForwardsCone = 4;
    
    public static final int kUnknownExtendKey = -1;
    public static final int k1stLevelBackCone = 0;
    public static final int k1stLevelBackCube = 1;
    public static final int k1stLevelForwardCone = 2;
    public static final int k1stLevelForwardCube = 3;
    public static final int k2ndLevelCone = 4;
    public static final int k2ndLevelCube = 5;
    public static final int k3rdLevelCone = 6;
    public static final int k3rdLevelCube = 7;

    public static final int kIntakeStage = 0;
    public static final int kExtendStage = 1;
    public static final int kDropStage = 2;

    public static final int kLeftPickPosition = 1;
    public static final int kRightPickPosition = 2;

    public static final int kLeftPlacePosition = 1;
    public static final int kMiddlePlacePosition = 2;
    public static final int kRightPlacePosition = 3;

    public static final int kRightGrid = 1;
    public static final int kMiddleGrid = 2;
    public static final int kLeftGrid = 3;

    
  }

  public static final class BlinkinConstants {
    public static final int kBlinkinPortId = 9; /* PWM input port on RoboRio */

    /*
     * The Blinkin module been setup with the following colors:
     * Color 1: Yellow (Cone)
     * Color 2: Violet (Cube)
     */

    public static final double kBlinkinConeGround = 0.69; // Yellow
    public static final double kBlinkinConeShelf = 0.11; // Color 1 Pattern Breath Fast
    public static final double kBlinkinConeDownGround = -0.01; // Color 1 Pattern Larson Scanner

    public static final double kBlinkinCubeGround = 0.91; // Violet
    public static final double kBlinkinCubeShelf = 0.31; // Color 2 Pattern Breath Fast
    public static final double kBlinkinCubeBackShelf = 0.19; // Color 2 Pattern Larson Scanner

    public static final double kBlinkinHasItem = 0.77;

    public static final double kBlinkinBalanceCelebrate = -0.57; // Fire Large, flexing, use only when balanced

    public static final double kBlinkinUnknownIntakeState = 0.62; /* Green */
  }

  public static final class CameraConstants{
    public static final boolean kDriveCam = true;
    public static final boolean kLimelight = false;
    public static final int kDriveCamIndex = 2;
    public static final int kLimelightIndex = 1;
  }
}