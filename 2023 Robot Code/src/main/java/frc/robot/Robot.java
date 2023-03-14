//General Imports
package frc.robot;
import java.lang.Math;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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

//Timer Imports
import edu.wpi.first.wpilibj.Timer;

//Camera Imports
import edu.wpi.first.cameraserver.CameraServer;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public class Robot extends TimedRobot {
  private RobotDrivetrain drivetrain = new RobotDrivetrain();
  private XboxController controller0 = new XboxController(0);
  private XboxController controller1 = new XboxController(1);
  
  //The navX
  private AHRS navX = new AHRS(SPI.Port.kMXP);
  private boolean autoBalanceXMode;
  private static final double balanceThreshold = 5;
  private boolean autoAngle;

  //Arm
  private CANSparkMax armMotor = new CANSparkMax(6, MotorType.kBrushless);
  private RelativeEncoder encoderArm = armMotor.getEncoder();

  //Claw
  private Solenoid claw = new Solenoid(10, PneumaticsModuleType.REVPH, 0);
  private Solenoid clawDeploy = new Solenoid(10, PneumaticsModuleType.REVPH, 1);

  //Timer
  private double startTime;
  private double time = Timer.getFPGATimestamp();


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //This function is run when the robot is first started up and should be used for any initialization code.
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //This function is called every 20 ms, no matter the mode.
  @Override
  public void robotPeriodic() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void autonomousInit() {
    drivetrain.setEncoderReset();
    navX.zeroYaw();
    drivetrain.encoderInfo();
    robotAttitude();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void autonomousPeriodic(){
    claw.set(true);//close
    claw.set(false);//open
    armMotor.set(0.25);
    boolean HDriveworks = true;

    boolean areWeAutoBalance = false;

    boolean straight = true;
    boolean right = true;
    boolean left = true;
    drivetrain.encoderInfo();
    robotAttitude();
    drivetrain.encoderInfo();
    while(time - startTime < 15){
      if (straight){
        if (drivetrain.getLeftEncoderPosition() > distToRevs(1)){
          drivetrain.curvatureDrive(-0.05, 0);
        }
        else if(drivetrain.getLeftEncoderPosition() < distToRevs(137.865 + (85.13/3))){
          drivetrain.curvatureDrive(0.15, 0);
        }
        else if(drivetrain.getLeftEncoderPosition() > distToRevs((137.865 + (85.13/3)) - ((85.13/3) + (76.125/2)))){
          drivetrain.curvatureDrive(-0.15, 0);
        }
        else if(areWeAutoBalance){
          AutoBalance();
        }
        else 
          drivetrain.curvatureDrive(0,0);
      }
      else if(right){
        if (drivetrain.getLeftEncoderPosition() > distToRevs(1)){
          drivetrain.curvatureDrive(-0.05, 0);
        }
        else if(drivetrain.getLeftEncoderPosition() < distToRevs(137.865 + (85.13/3))){
          drivetrain.curvatureDrive(0.15, 0);
        }
        if (HDriveworks){
          if(areWeAutoBalance){
            if(drivetrain.getHDriveEncoderPosition() > distToRevs(-114.095)){
              drivetrain.hDriveMovement(-1);
            }
            else if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(-0.15, 0);
            }
          }
          else
            drivetrain.curvatureDrive(0, 0);
        }
        else{
          if(areWeAutoBalance){
            angleRotation(270);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(86.39)){
              drivetrain.curvatureDrive(distToRevs(0.15), 0);
            }
            angleRotation(270);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(0.15, 0);
            }
            AutoBalance();
          }
          else
            drivetrain.curvatureDrive(0,0);
        }
      }
      else if(left){
        if (drivetrain.getLeftEncoderPosition() > distToRevs(1)){
          drivetrain.curvatureDrive(-0.05, 0);
        }
        else if(drivetrain.getLeftEncoderPosition() < distToRevs(137.865 + (85.13/3))){
          drivetrain.curvatureDrive(0.15, 0);
        }
        if (HDriveworks){
          if(areWeAutoBalance){
            if(drivetrain.getHDriveEncoderPosition() < distToRevs(86.39)){
              drivetrain.hDriveMovement(1);
            }
            else if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(-0.15, 0);
            }
          }
          else
            drivetrain.curvatureDrive(0, 0);
        }
        else{
          if(areWeAutoBalance){
            angleRotation(90);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(86.39)){
              drivetrain.curvatureDrive(distToRevs(0.15), 0);
            }
            angleRotation(90);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(0.15, 0);
            }
            AutoBalance();
          }
          else
            drivetrain.curvatureDrive(0,0);
        }
      }
      else 
        drivetrain.curvatureDrive(0,0);
    }
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void teleopInit() {
    drivetrain.setEncoderReset();
    navX.zeroYaw();
    navX.resetDisplacement();
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  public double getArmEncoderPosition(){
    return encoderArm.getPosition();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void teleopPeriodic() {
    drivetrain.encoderInfo();
    
    //Information for Pitch, Yaw, Speed, and Position
    robotAttitude();
    
    //Curvature Drive  
    double forwardSpeed = controller0.getRightTriggerAxis();
    double reverseSpeed = controller0.getLeftTriggerAxis();
    boolean leftSpeed = controller0.getLeftBumper(); //H-Drive
    boolean rightSpeed = controller0.getRightBumper(); //H-Drive
    double turn = controller0.getLeftX();
    
    if (forwardSpeed > 0){
      drivetrain.curvatureDrive(.5*forwardSpeed, -1*turn);
    }
    else if (reverseSpeed > 0){
      drivetrain.curvatureDrive(-.3*reverseSpeed, -.5*turn);
    }
    else{
      drivetrain.curvatureDrive(0,0);
    }    
    
    //H-Drive
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
    
    //Arm Code
    boolean armUp = controller1.getRightBumper();
    boolean armDown = controller1.getLeftBumper();
    SmartDashboard.putNumber("Encoder Position Arm", encoderArm.getPosition());
    
    if(armUp){
      armMotor.set(0.6);    
    }
    else if(armDown) {
      armMotor.set(-.6);
    }
    else {
      armMotor.set(0);
    }

    //Claw Code
    boolean clawOpen = controller1.getAButton();
    boolean clawDown = controller1.getXButton();
    boolean clawUp = controller1.getYButton();

    if (clawOpen) {
      claw.set(true);
    }
    else{
      claw.set(false);
    }

    if(clawDown){
      clawDeploy.set(true);
    }
    if(clawUp){
      clawDeploy.set(false);
    } 
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  public void robotAttitude(){
    SmartDashboard.putNumber("Time", time);
    SmartDashboard.putNumber("Pitch Angle", navX.getPitch());
    SmartDashboard.putNumber("Yaw Angle", navX.getAngle());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  public double distToRevs(double distance){
    double revolutions = 0.552*distance - 4.41;
    return revolutions;
  }

  ////////////////////////////////////////////////   NOT BEING USED   //////////////////////////////////////////////////////
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