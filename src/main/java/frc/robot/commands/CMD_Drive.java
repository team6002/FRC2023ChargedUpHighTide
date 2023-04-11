package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_IntakeCamera;
// import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_Drive extends CommandBase {

  private final SUB_Drivetrain m_drivetrain;
  private final CommandXboxController controller;
  private final SUB_Limelight m_limelight;
  private final SUB_IntakeCamera m_intakeCam;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(0.5);
  // private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(0.5);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.2);
  public boolean fieldMode = false;
  double deadzone = 0.2;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double turn = 0;        //variable for turning movement
  final double limelightAngleThreshold = LimeLightConstants.klimelightAngleThreshold;
  final double limelightAdjustRotKp = LimeLightConstants.klimelightAdjustRotKp;
  final double limelightAdjustStrafeKp = LimeLightConstants.klimelightAdjustStrafeKp;

  public CMD_Drive(SUB_Drivetrain m_drivetrain, CommandXboxController m_driverControllerTrigger, SUB_Limelight p_limelight, SUB_IntakeCamera p_intakeCam) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);

    this.controller = m_driverControllerTrigger;
    this.m_limelight = p_limelight;
    this.m_intakeCam = p_intakeCam;
  }

  @Override
  public void initialize() {
  }
 
  @Override
  public void execute() {
    // y = controller.getLeftY();

    // x = controller.getLeftX();

    // turn = controller.getRightX();
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    // final var xSpeed = xspeedLimiter.calculate(modifyAxis(-controller.getLeftY()))
    //     * DriveConstants.kMaxSpeedMetersPerSecond;

    // final var xSpeed = xspeedLimiter.calculate(MathUtil.applyDeadband(-controller.getLeftY(), deadzone));
    // final var xSpeed = MathUtil.applyDeadband(-controller.getLeftY(), deadzone);  
    var xSpeed = MathUtil.applyDeadband(-controller.getLeftY(),deadzone);
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    // final var ySpeed = yspeedLimiter.calculate(MathUtil.applyDeadband(-controller.getLeftX(), deadzone));
    // final var ySpeed = MathUtil.applyDeadband(-controller.getLeftX(), deadzone);
    var ySpeed = MathUtil.applyDeadband(-controller.getLeftX(),deadzone);
 
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    
    
    // final var rot = rotLimiter.calculate(MathUtil.applyDeadband(-controller.getRightX(), deadzone));
    var rot = modifyAxis(MathUtil.applyDeadband(-controller.getRightX(), deadzone));

    /* Override driver rotation if AutoAlign is enabled. */
    if (this.controller.rightTrigger().getAsBoolean()) {
      // m_limelight.useConePipeline();
      
      if (m_limelight.hasTarget()) {
        double heading_error = m_limelight.getTargetTx();
        double strafe_error = m_limelight.getTargetTx();
        
        if (Math.abs(strafe_error) > limelightAngleThreshold) {
          // rot = -strafe_error * limelightAdjustKp;
        if (Math.abs(heading_error) > limelightAngleThreshold) {
          rot = -heading_error * limelightAdjustRotKp;
          // ySpeed = -strafe_error * limelightAdjustStrafeKp;
          SmartDashboard.putNumber("YSPEED", ((strafe_error * limelightAdjustStrafeKp) + Math.copySign(LimeLightConstants.klimelightAdjustRotKf, -heading_error)));
        }
        }
      }
    } else if (this.controller.leftTrigger().getAsBoolean()) {
      /* TODO: ALSO CHECK THAT WE'RE IN CUBE GROUND INTAKING STATE */
      if (m_intakeCam.hasTarget()) {
        double heading_error = m_intakeCam.getTxZero() - m_intakeCam.getTargetTx();

        if (Math.abs(heading_error) > 5) {
          rot = heading_error * 0.002;

          SmartDashboard.putNumber("IntakeCamSpeed", rot);
        }
      }
    }

    // SmartDashboard.putNumber("xspeed", xSpeed);
    // SmartDashboard.putNumber("yspeed", ySpeed);
    // SmartDashboard.putNumber("rotspeed", rot);
    // SmartDashboard.putNumber("yaxis", controller.getLeftY());
    // SmartDashboard.putNumber("x-axis", controller.getRightX());



    m_drivetrain.drive( xSpeed, ySpeed, rot,true,false);
    // m_drivetrain.drive( .4, 0, 0.0, true);
  }


  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0, true,false);
  }

  // private static double deadband(double value, double deadband) {
  //   if (Math.abs(value) > deadband) {
  //     if (value > 0.0) {
  //       return (value - deadband) / (1.0 - deadband);
  //     } else {
  //       return (value + deadband) / (1.0 - deadband);
  //     }
  //   } else {
  //     return 0.0;
  //   }
  // }

  private static double modifyAxis(double value) {
    double modifedValue;
    // Deadband
    // value = deadband(value, 0.2);

    // Square the axis
    modifedValue = value * value;
    modifedValue = Math.copySign(value, value);

    return modifedValue;
  }

}