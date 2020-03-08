/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  
  // Sparks variables
  private CANSparkMax frontLeft;
  private CANSparkMax rearLeft;
  private CANSparkMax frontRight;
  private CANSparkMax rearRight;
  private CANSparkMax intake;
  private CANSparkMax carry;
  
  // Drivetrain variables
  private SpeedControllerGroup left;
  private SpeedControllerGroup right;
  private DifferentialDrive robotDrive;

  // Movement variables
  private double forward;
  private double rotate;

  // PS4 variables
  private Joystick ps4;
  private double leftTrigger;
  private double rightTrigger;
  private double leftStickX;
  private double rightStickY;
  private boolean leftBumper;
  private boolean rightBumper;
  private boolean crossButton;

  // Camera server object
  private CameraServer server;

  /**
   * --Constants--
   */

  // Motor OI
  private static final int frontLeft_ID = 8;
  private static final int rearLeft_ID = 9;
  private static final int frontRight_ID = 3;
  private static final int rearRight_ID = 1;
  private static final int intake_ID = 7;
  private static final int carry_ID = 2;
  //private static final 

  // PS4 OI
  private static final int ps4_Port = 0;
  private static final int leftTrigger_Axis = 3;
  private static final int rightTrigger_Axis = 4;
  private static final int rightStickY_ID = 5;
  private static final Hand leftStickX_Hand = Hand.kLeft;
  private static final int leftBumper_ID = 5;
  private static final int rightBumper_ID = 6;
  private static final int crossButton_ID = 2;

  // Sensitivity
  private static final double forward_Sensitivity = 0.4;
  private static final double rotate_Sensitivity = 0.5;
  private static final double intake_Sensitivity = 0.5;
  private static final double carry_Sensitivity = 0.5;

  private final Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Assigning Spark ID
    frontLeft = new CANSparkMax(frontLeft_ID, MotorType.kBrushless);
    rearLeft = new CANSparkMax(rearLeft_ID, MotorType.kBrushless);
    frontRight = new CANSparkMax(frontRight_ID, MotorType.kBrushless);
    rearRight = new CANSparkMax(rearRight_ID, MotorType.kBrushless);
    intake = new CANSparkMax(intake_ID, MotorType.kBrushless);
    carry = new CANSparkMax(carry_ID, MotorType.kBrushless);

    // Instantiating Drivetrain
    left = new SpeedControllerGroup(frontLeft, rearLeft);
    right = new SpeedControllerGroup(frontRight, rearRight);
    robotDrive = new DifferentialDrive(left, right);

    // Assigning PS4 Controller ID
    ps4 = new Joystick(ps4_Port);

    // Initiates and Displays USB Camera
    server = CameraServer.getInstance();
    server.startAutomaticCapture();
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (timer.get() < 2.0) {
      robotDrive.arcadeDrive(0.5, 0.0); // drives forwards half speed
    } else {
      robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    
    // Assigns PS4 OI
    leftTrigger = ps4.getRawAxis(leftTrigger_Axis);
    rightTrigger = ps4.getRawAxis(rightTrigger_Axis);
    leftStickX = ps4.getX(leftStickX_Hand);
    rightStickY = ps4.getRawAxis(rightStickY_ID);
    leftBumper = ps4.getRawButton(leftBumper_ID);
    rightBumper = ps4.getRawButton(rightBumper_ID);
    crossButton = ps4.getRawButton(crossButton_ID);

    // Movement
    forward = (rightTrigger - leftTrigger) * forward_Sensitivity;
    rotate = leftStickX * rotate_Sensitivity;
    robotDrive.arcadeDrive(forward, rotate);

    // Button Mapping
    if (crossButton) {
      forward *= 2;
    }

    if (leftBumper) {
      intake.set(-intake_Sensitivity);
    } else {
      intake.stopMotor(); // stop intake
    }

    if (rightBumper) {
      intake.set(intake_Sensitivity);
    } else {
      intake.stopMotor(); // stop intake
    }

    if (rightStickY != 0) {
      carry.set(rightStickY * -carry_Sensitivity);
    }

    //Limelight Data Start
// get the default instance of NetworkTables
NetworkTableInstance inst = NetworkTableInstance.getDefault();
// get a reference to the subtable called "datatable"
NetworkTable table = inst.getTable("limelight");
 
inst.startClientTeam(3535); // Make sure you set this to your team number
 
inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
 
// NetworkTableEntry TeamEntry = table.getEntry("tx");
NetworkTableEntry xEntry = table.getEntry("tx");
NetworkTableEntry yEntry = table.getEntry("ty");
NetworkTableEntry aEntry = table.getEntry("ta");
NetworkTableEntry lEntry = table.getEntry("tl");
NetworkTableEntry vEntry = table.getEntry("tv");
NetworkTableEntry sEntry = table.getEntry("ts");
 
NetworkTableEntry tshortEntry = table.getEntry("tshort");
NetworkTableEntry tlongEntry = table.getEntry("tlong");
NetworkTableEntry thorEntry = table.getEntry("thor");
NetworkTableEntry tvertEntry = table.getEntry("tvert");
NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
NetworkTableEntry camtranEntry = table.getEntry("camtran");
NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
 
// double tx = xEntry.getDouble(0.0);
double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
double ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
double ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
double tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                   // latency.
double tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
double ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
 
// double tshort = tshortEntry.getString(); // Sidelength of shortest side of
// the fitted bounding box (pixels)
// double tlong = tlong; // Sidelength of longest side of the fitted bounding box
// (pixels)
// double thor = thor; // Horizontal sidelength of the rough bounding box (0 -
// 320 pixels)
// double tvert = tvert; // Vertical sidelength of the rough bounding box (0 -
// 320 pixels)
// double getpipe = getpipe; // True active pipeline index of the camera (0 .. 9)
// double camtran = camtran; // Results of a 3D position solution, 6 numbers:
//Translation (x,y,y) Rotation(pitch,yaw,roll);
 
//ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
ledModeEntry.setNumber(1); // force off
//ledModeEntry.setNumber(2); // force blink
//ledModeEntry.setNumber(3); // force on
 
//System.out.println("X: " + tx);
//System.out.println("Y: " + ty);
//System.out.println("A: " + ta);
//System.out.println("L: " + tl);
//System.out.println("V: " + tv);
//System.out.println("S: " + tv);
 
// post to smart dashboard periodically
SmartDashboard.putNumber("Limelight X", tx);
SmartDashboard.putNumber("Limelight Y", ty);
SmartDashboard.putNumber("Limelight Area", ta);
SmartDashboard.putNumber("Limelight Latency", tl);
SmartDashboard.putNumber("Limelight Valid Target", tv);
SmartDashboard.putNumber("Limelight Skew", ts);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
