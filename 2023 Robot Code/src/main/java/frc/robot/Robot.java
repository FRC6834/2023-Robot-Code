// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private RobotDrivetrain drivetrain = new RobotDrivetrain();
  private XboxController controller0 = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    drivetrain.setEncoderReset();
  }

  /** This function is called periodically during autonomous. */
  
  //Distance in inches to revolutions for neo motor
  public double distToRevs(double distance){
    double revolutions = 0.552*distance - 4.41;
    return revolutions;

  }
  @Override
  public void autonomousPeriodic() {
    
    //Made it so only distance needs to be changed to go foward or backwards.
    double distance = 60;
    double speed = 0.15;
    drivetrain.encoderInfo();
    
  if (distance > 0 && drivetrain.getLeftEncoderPosition() <= distToRevs(distance)){
    drivetrain.curvatureDrive(-1*speed, 0);
  }
  else if(distance < 0 && drivetrain.getLeftEncoderPosition() >= distToRevs(distance)){
    drivetrain.curvatureDrive(-1*speed, 0);
  }



  //FIX THIS DOM
  //YOUR CODE IS DUMB!!!!!
  //BE BETTER
  //THIS IS WHY I'M STEALING YOUR JOB!
  /*else{
      drivetrain.curvatureDrive(0, 0);
    if (drivetrain.getLeftEncoderPosition() >= -20 && drivetrain.getRightEncoderPosition() <= 20){
      drivetrain.dPadGetter(270);
    }
    else{
      drivetrain.curvatureDrive(0,0);
      if (drivetrain.getLeftEncoderPosition() <= distToRevs(distance)){
        drivetrain.curvatureDrive(speed, 0);
     }
     else{
        drivetrain.curvatureDrive(0, 0);
     }
    }
  }*/
  
    
    /*if (drivetrain.getLeftEncoderPosition() <= 62 && drivetrain.getRightEncoderPosition() <= 62){
	    drivetrain.curvatureDrive(0.15, 0);
    }
    else
	    drivetrain.curvatureDrive(0, 0);
*/
  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drivetrain.setEncoderReset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //D-pad functionality works regardless of drive type chosen
    
    //Tank Drive
    //Need y-axis for each stick
    //Hand.kLeft gives the left analog stick and Hand.kRight gives the right analog stick
    //Speeds are currently set at 50%
    //drivetrain.tankDrive(-0.5*controller.getLeftY(), -0.5*controller.getRightY()); 
    drivetrain.encoderInfo();
    //Curvature Drive  
    double fSpeed = controller0.getRightTriggerAxis(); //forward speed from right trigger
    double rSpeed = controller0.getLeftTriggerAxis(); //reverse speed from left trigger
    double turn = controller0.getLeftX(); //gets the direction from the left analog stick
    
    if (fSpeed > 0){
      drivetrain.curvatureDrive(fSpeed, -1*turn);
    }
    else if (rSpeed > 0){
      drivetrain.curvatureDrive(-1*rSpeed, -1*turn);
    }
    else{
      drivetrain.curvatureDrive(0,0);
    }    
    
    //D-Pad controls for fine movements
    int dPad = controller0.getPOV(); //scans to see which directional arrow is being pushed
    drivetrain.dPadGetter(dPad);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
//pilk is good
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
