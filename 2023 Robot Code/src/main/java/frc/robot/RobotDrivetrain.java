package frc.robot;

//Imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
/*Must install rev robotics library in order to use SPARK Max speed controllers. 
 *Follow directions on site below.
 *https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#java-api
 *Help docs - https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html
*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//The goal of this class is to create methods that can be used to operate the robot's drivetrain.
//This will clean up the code in Robot.java and will allow for easier future fixes.
public class RobotDrivetrain {
  
  //Creates SPARK MAX objects for each speed controller in the drivetrain.
  //Orientation assumes killswitch is at front of robot
  //The first parameter refers to the CAN ID - Use Rev Tool to determine CAN IDs
  //MotorType.kBrushless MUST be used when using NEO brushless motors
  //Use REV Tool to ensure motor controllers are flashed for brushless
  private CANSparkMax leftFront = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax leftRear = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rightRear = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax hDrive = new CANSparkMax(5, MotorType.kBrushless);


  /*
   * Returns an object for interfacing with the hall sensor integrated into a brushless motor, which is connected to the 
   * front port of the SPARK MAX.
  */
  private RelativeEncoder encoderLeft = leftFront.getEncoder();
  private RelativeEncoder encoderRight = rightFront.getEncoder();
  private RelativeEncoder encoderHDrive = hDrive.getEncoder();

  
  //Groups left side speed controllers together and right side speed controllers together
  //This is cleaner than setting up two different differential drives
  private MotorControllerGroup driveLeft = new MotorControllerGroup(leftFront, leftRear);
  private MotorControllerGroup  driveRight = new MotorControllerGroup(rightFront, rightRear);
  //The DifferentialDrive class gives us access to the arcade drive, tank drive, and curvature drive methods.
  private DifferentialDrive robotDrive = new DifferentialDrive(driveLeft, driveRight);

  //Constructor is called in Robot.java to create RobotDrivetrain objects.
  //Nothing needs to happen in the constructor for our purposes
  public RobotDrivetrain(){
    rightFront.setInverted(true);
    rightRear.setInverted(true);
  }
  
  //Not currently being used
  //xSpeed gives the forward/backward
  //zRotation gives the left/right
  //Values run from -1.0 to 1.0
  //Use a value of 0 if you want robot to be stationary in particular direction
  public void arcadeDrive(double xSpeed, double zRotation){
    robotDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void hDriveMovement(double xSpeed){
    hDrive.set(xSpeed);
  }
  //used in Dpad controls and autonomous mode
  //leftSpeed (-1.0 to 1.0) - comes from left stick
  //rightSpeed (-1.0 to 1.0) - comes from right stick
  public void tankDrive(double leftSpeed, double rightSpeed){
    robotDrive.tankDrive(leftSpeed, rightSpeed);
  }

  //Method for curvature drive - used for main drivetrain controls on robot
  //Right side motor controllers are inverted to ensure that both sides of the bot move in the same direction when desired
  //xSpeed (0 to 1.0) to go forward and (-1.0 to 0) to move backward
  //zRotation (-1.0 to 1.0) controls direction
  public void curvatureDrive(double xSpeed, double zRotation){
    rightFront.setInverted(true);
    rightRear.setInverted(true);
    robotDrive.curvatureDrive(xSpeed, zRotation, false);
  }

  /*
  * D-Pad setup - these are used for small movements
  * 0 = up arrow (forward)
  * 90 = right arrow (right)
  * 180 = down arrow (backward)
  * 270 = left arrow (left)
  * speeds can be adjusted
  * Setting equal to 0 for left/right ensures it stays in place
  */

  public void dPadGetter(int dPad){
    rightFront.setInverted(true);
    rightRear.setInverted(true);
    if (dPad==0){
      robotDrive.tankDrive(0.2, 0.2); //forward
    }
    if (dPad==90){
      robotDrive.tankDrive(0.2, -0.2); //right
    }
    if (dPad==180){
      robotDrive.tankDrive(-0.2, -0.2); //reverse
    }
    if (dPad==270){
      robotDrive.tankDrive(-0.2, 0.2); //left
    }
  } 
  
  
  public void encoderInfo(){
    SmartDashboard.putNumber("Encoder Position Left", encoderLeft.getPosition());
    SmartDashboard.putNumber("Encoder Position Right", encoderRight.getPosition());
    SmartDashboard.putNumber("Encoder Velocity Left", encoderLeft.getVelocity());
    SmartDashboard.putNumber("Encoder Velocity Right", encoderRight.getVelocity());
    SmartDashboard.putNumber("Encoder Velocity H-Drive", encoderHDrive.getVelocity());
    SmartDashboard.putNumber("Encoder Position H-Drive", encoderHDrive.getPosition());
  }
  public void setEncoderReset(){
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
    encoderHDrive.setPosition(0);
  }
  public double getLeftEncoderPosition(){
    return encoderLeft.getPosition();
  }
  public double getRightEncoderPosition(){
    return encoderRight.getPosition();
  }

  public double getHDriveEncoderPosition(){
    return encoderHDrive.getPosition();
  }
  /*public void encoderAutonomousStart(){
    setEncoderReset();
  if (getLeftEncoderPosition() <= 62 && getRightEncoderPosition() <= 62){
    curvatureDrive(0.15, 0);
  }
  else
    curvatureDrive(0, 0);
  setEncoderReset();
  if (getLeftEncoderPosition() >= -20 && getRightEncoderPosition() <= 20){
  dPadGetter(270);
  }
  else
    curvatureDrive(0,0);
  setEncoderReset();
  
  if (getLeftEncoderPosition() <= 62 && getRightEncoderPosition() <= 62){
    curvatureDrive(0.15, 0);
  }
  else
    curvatureDrive(0, 0);
}*/
}