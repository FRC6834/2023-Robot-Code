//General Imports
package frc.robot;
import java.lang.Math;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

//navX Imports: https://dev.studica.com/releases/2023/NavX.json
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

//Pneumatic Imports
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//Do we need these???
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.PneumaticHub;

//Camera imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

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
  //The navX
  private AHRS navX = new AHRS(SPI.Port.kMXP);
  private boolean autoBalanceXMode;
  private static final double balanceThreshold = 5;
  private boolean autoAngle;

  private CANSparkMax armMotor = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax claw = new CANSparkMax(7, MotorType.kBrushed);
  private RelativeEncoder encoderArm = armMotor.getEncoder();
  //private RelativeEncoder encoderClaw = claw.getEncoder();

  // Pneumatics Initialization
  private final Solenoid s1 = new Solenoid(10, PneumaticsModuleType.REVPH, 0);

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);
    CameraServer.startAutomaticCapture();
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
    navX.zeroYaw();
  }

  /** This function is called periodically during autonomous. */
  
  //Distance in inches to revolutions for neo motor
  public double distToRevs(double distance){
    double revolutions = 0.552*distance - 4.41;
    return revolutions;
  }

  @Override
  public void autonomousPeriodic() {
    //SmartDashboard.putNumber("Pitch Angle", navX.getPitch());
    //Made it so only distance needs to be changed to go foward or backwards.
    /*double distance = 60;
    double speed = 0.15;
    drivetrain.encoderInfo();
    
  if (distance > 0 && drivetrain.getLeftEncoderPosition() <= distToRevs(distance)){
    drivetrain.curvatureDrive(-1*speed, 0);
  }
  else if(distance < 0 && drivetrain.getLeftEncoderPosition() >= distToRevs(distance)){
    drivetrain.curvatureDrive(-1*speed, 0);
  }*/
    drivetrain.encoderInfo();
    SmartDashboard.putNumber("Pitch Angle", navX.getPitch());
    SmartDashboard.putNumber("Yaw Angle", navX.getAngle());
    AutoBalance();
    angleRotation(180);
  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drivetrain.setEncoderReset();
    navX.zeroYaw();
  }

  public double getArmEncoderPosition(){
    return encoderArm.getPosition();
  }
  /*
  public double getClawEncoderPosition(){
    return encoderClaw.getPosition();
  }
  */

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
    SmartDashboard.putNumber("Pitch Angle", navX.getPitch());
    SmartDashboard.putNumber("Yaw Angle", navX.getAngle());
    //Curvature Drive  
    double forwardSpeed = controller0.getLeftTriggerAxis(); //forward speed from right trigger
    double reverseSpeed = controller0.getRightTriggerAxis(); //reverse speed from left trigger
    boolean leftSpeed = controller0.getLeftBumper(); //right direction from right bumper
    boolean rightSpeed = controller0.getRightBumper(); //left direction from left bumper
    double turn = -1*controller0.getLeftX(); //gets the direction from the left analog stick
    
    if (forwardSpeed > 0){
      drivetrain.curvatureDrive(.5*forwardSpeed, -1*turn);
    }
    else if (reverseSpeed > 0){
      drivetrain.curvatureDrive(-.5*reverseSpeed, -1*turn);
    }
    else{
      drivetrain.curvatureDrive(0,0);
    }    
    
    //h-drive
    if (leftSpeed){
      drivetrain.hDriveMovement(-0.5);
    }
    else if (rightSpeed){
      drivetrain.hDriveMovement(0.5);
    }
    else{
      drivetrain.hDriveMovement(0);
    }

    //D-Pad controls for fine movements
    int dPad = controller0.getPOV(); //scans to see which directional arrow is being pushed
    drivetrain.dPadGetter(dPad);
    
    /* this function moves the arm up and down, 
    it has a set limit that needs to be found so we can input it 
    theses codes are thoughts of possible code for the arm, don't know how to access motor currently
    so it's just a thought
    THIS DOES NOT WORK YET!!! BE CAREFUL!!!
    */
    
    //arm code
    boolean armUp = controller0.getYButton();
    boolean armDown = controller0.getXButton();
    double upLimit = 50.0;
    double downLimit = 100.0;
    SmartDashboard.putNumber("Encoder Position Arm", encoderArm.getPosition());
    
    if(armUp){
      armMotor.set(.3);
    }
    else if(armDown){
      armMotor.set(-.2);
    }
    else{
      armMotor.set(0);
    }

    /*
    if (getArmEncoderPosition() < upLimit){

      if(armUp){
        armMotor.set(-0.25);
      }
      else {
        armMotor.set(0);
      }
    }
    if (getArmEncoderPosition() > downLimit) {
      if(armDown){
        armMotor.set(0.25);
      }
      else {
        armMotor.set(0);
      }
    }
    */


    //Electric claw
    boolean clawOpen = controller0.getAButton();
    boolean clawClose = controller0.getBButton();
    double openLimit = 15.0;
    double closeLimit = 10.0;
    if(clawOpen){
      claw.set(1);
    }
    else if(clawClose){
      claw.set(-1);
    }
    else{
      claw.set(0);
    }

    //pneumaticsclaw code
    boolean clawPNClose = controller0.getAButton();
    if (clawPNClose) {
      s1.set(true);
    }
    else{
      s1.set(false);
    }
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

  //AutoBalance for autonomous period 
  //Should make the robot speed slow as it gets closer to equilibrium
  //It seems to be unable to switch from Teleop to Auto and Back
  public void AutoBalance(){
    double pitchDegrees = navX.getPitch();
    if (!autoBalanceXMode && (Math.abs(pitchDegrees) > Math.abs(balanceThreshold))){
      autoBalanceXMode = true;
    }
    if (autoBalanceXMode && (Math.abs(pitchDegrees) <= Math.abs(balanceThreshold))){
      autoBalanceXMode = false;
    } 
    if (autoBalanceXMode){
      double pitchAngleRadians = pitchDegrees * (Math.PI / 180.0);
      double balanceSpeed = Math.sin(pitchAngleRadians) * -1; 
      drivetrain.curvatureDrive(balanceSpeed, 0);
    }
  }
  
  public void angleRotation(double angle){
    double yawDegrees = navX.getAngle();
    double angleSpeed = 0;
    if (!autoAngle && (yawDegrees < angle)){
      autoAngle = true;
    }
    if (autoAngle && (yawDegrees >= angle)){
      autoAngle = false;
    }
    if(autoAngle){
      double yawRadians = yawDegrees * (Math.PI / 180.0);
      if(angle > 90){
        angleSpeed = 0.5 * (Math.cos(.5*yawRadians) + 0.29);
      }
      else if (angle <= 90 ){
        angleSpeed = 0.5 * (Math.cos(yawRadians) + 0.2);
      }
      drivetrain.tankDrive(-angleSpeed, angleSpeed);
    }
  }
}
