//General Imports
package frc.robot;
import java.lang.Math;
import java.sql.Time;

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
  private static final double kOffBalanceAngleThresholdDegrees = 2;
  private static final double kOonBalanceAngleThresholdDegrees  = 0;
  private boolean autoAngle;

  //Arm
  private CANSparkMax armMotor = new CANSparkMax(6, MotorType.kBrushless);
  private RelativeEncoder encoderArm = armMotor.getEncoder();

  //Claw
  private Solenoid claw = new Solenoid(10, PneumaticsModuleType.REVPH, 0);
  private Solenoid clawDeploy = new Solenoid(10, PneumaticsModuleType.REVPH, 1);

  //Timer
  private double startTime;


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
    startTime = Timer.getFPGATimestamp();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void autonomousPeriodic(){
    //claw.set(true);//close
    //claw.set(false);//open
    //armMotor.set(0.25);

    double time = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Time", time);
    boolean HDriveworks = true;

    boolean areWeAutoBalance = true;

    boolean straight = true;
    boolean right = false;
    boolean left = false;
    drivetrain.encoderInfo();
    robotAttitude();
    if (straight){
      if(time - startTime < 2){
        clawDeploy.set(true);
        if (drivetrain.getLeftEncoderPosition() < distToRevs(7)){
          drivetrain.curvatureDrive(0.25, 0);
          claw.set(true);
        }
      }
      else if (time - startTime < 5){
        claw.set(false);
        clawDeploy.set(false);
        if(drivetrain.getLeftEncoderPosition() > distToRevs(-1*(137.865 + (85.13/3)))){
          drivetrain.curvatureDrive(-0.35, 0);
        }
      }
      else if (time - startTime < 8){
        if(drivetrain.getLeftEncoderPosition() < distToRevs((-1*(137.865 + (85.13/3))) + ((85.13/3) + (76.125/2)))){
          drivetrain.curvatureDrive(0.45, 0);
        }
      }
      else if(time - startTime < 15){
        if(areWeAutoBalance){
          AutoBalance();
        }
        else  
          drivetrain.curvatureDrive(0,0);
      }
    }
    /**else if(right){
      if(time - startTime < 3){
        clawDeploy.set(false);
        if (drivetrain.getLeftEncoderPosition() > distToRevs(10)){
          drivetrain.curvatureDrive(0.15, 0);
        }
        claw.set(true);
      }
      else if (time - startTime < 6){
        claw.set(false);
        clawDeploy.set(true);
        if(drivetrain.getLeftEncoderPosition() > distToRevs(-1*(137.865 + (85.13/3)))){
          drivetrain.curvatureDrive(-0.65, 0);
        }
      }
      else{
        if(HDriveworks){
          if (time - startTime < 8){
            if(drivetrain.getHDriveEncoderPosition() > distToRevs(-114.095)){
              drivetrain.hDriveMovement(-1);
            }
          }
          else if(time - startTime < 11){
            if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(-0.15, 0);
            }
          }
          else if(time - startTime < 15){
            if(areWeAutoBalance){
              AutoBalance();
            }
            else
              drivetrain.curvatureDrive(0, 0);
          }
        }
        else{
          if(time - startTime < 8){
            angleRotation(270);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(86.39)){
              drivetrain.curvatureDrive(distToRevs(0.15), 0);
            }
          }
          else if(time - startTime < 11){
            angleRotation(270);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(0.15, 0);
            }
          }
          else if(time - startTime < 15){
            if (areWeAutoBalance){
              AutoBalance();
            }
            else
              drivetrain.curvatureDrive(0,0);
          }
        }
      }
    }
    else if(left){
      if(time - startTime < 3){
        clawDeploy.set(false);
        if (drivetrain.getLeftEncoderPosition() > distToRevs(10)){
          drivetrain.curvatureDrive(0.15, 0);
        }
        claw.set(true);
      }
      else if (time - startTime < 6){
        claw.set(false);
        clawDeploy.set(true);
        if(drivetrain.getLeftEncoderPosition() > distToRevs(-1*(137.865 + (85.13/3)))){
          drivetrain.curvatureDrive(-0.65, 0);
        }
      }
      else{
        if (HDriveworks){
          if (time - startTime < 8){
            if(drivetrain.getHDriveEncoderPosition() < distToRevs(86.39)){
              drivetrain.hDriveMovement(-0.5);
            }
          }
          else if(time - startTime < 11){
            if(drivetrain.getLeftEncoderPosition() < distToRevs(-53.752)){
              drivetrain.curvatureDrive(0.15, 0);
            }
          }
          else if(time - startTime < 15){
            if(areWeAutoBalance){
              AutoBalance();
            }
            else
              drivetrain.curvatureDrive(0,0);
          }
          else
            drivetrain.curvatureDrive(0, 0);
        }
        else{
          if(time - startTime < 7){
            angleRotation(2);
          }
          else if(time -startTime < 9){
            if(drivetrain.getLeftEncoderPosition() < distToRevs(86.39)){
              drivetrain.curvatureDrive(distToRevs(0.15), 0);
            }
          }
          else if (time - startTime < 12){
            angleRotation(90);
            if(drivetrain.getLeftEncoderPosition() < distToRevs(53.752)){
              drivetrain.curvatureDrive(0.15, 0);
            }
          }
          else if(time - startTime < 15){
            if(areWeAutoBalance){
              AutoBalance();
            }
            else
              drivetrain.curvatureDrive(0,0);
          }
          else
            drivetrain.curvatureDrive(0,0);
        }
      }
    }**/
    else 
      drivetrain.curvatureDrive(0,0);
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
    boolean autoBalance = controller0.getAButton();
    
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
      clawDeploy.set(false);
    }
    if(clawUp){
      clawDeploy.set(true);
    } 

    // when drive team wants to auto balance
    if(autoBalance){
      AutoBalance();
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  public void AutoBalance(){
    double pitchAngleDegrees = navX.getPitch();
    if ( !autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))){
      autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode && (Math.abs(pitchAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))){
      autoBalanceXMode = false;
    }
    if ( autoBalanceXMode ){
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      drivetrain.curvatureDrive(Math.sin(pitchAngleRadians) * -1,0);
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
      else {
        angleSpeed = 0.5 * (Math.cos(yawRadians) + 0.2);
      }
      drivetrain.tankDrive(-angleSpeed, angleSpeed);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  public void robotAttitude(){
    SmartDashboard.putNumber("Time", startTime);
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