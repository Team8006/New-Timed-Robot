/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  
  // Sparks variables
  private Spark frontLeft;
  private Spark rearLeft;
  private Spark frontRight;
  private Spark rearRight;
  private Spark intake;
  private Spark carry;
  
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
  private static final int frontLeft_Port = 8;
  private static final int rearLeft_Port = 9;
  private static final int frontRight_Port = 0;
  private static final int rearRight_Port = 1;
  private static final int intake_Port = 7;
  private static final int carry_Port = 2;

  // PS4 OI
  private static final int ps4_Port = 0;
  private static final int leftTrigger_Axis = 3;
  private static final int rightTrigger_Axis = 4;
  private static final Hand rightStickY_Hand = Hand.kRight;
  private static final Hand leftStickX_Hand = Hand.kLeft;
  private static final int leftBumper_Port = 5;
  private static final int rightBumper_Port = 6;
  private static final int crossButton_Port = 2;

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
    frontLeft = new Spark(frontLeft_Port);
    rearLeft = new Spark(rearLeft_Port);
    frontRight = new Spark(frontRight_Port);
    rearRight = new Spark(rearRight_Port);
    intake = new Spark(intake_Port);
    carry = new Spark(carry_Port);

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

    // Assigns Buttons and Trigger IDs
    leftTrigger = ps4.getRawAxis(leftTrigger_Axis);
    rightTrigger = ps4.getRawAxis(rightTrigger_Axis);
    leftStickX = ps4.getX(leftStickX_Hand);
    rightStickY = ps4.getY(rightStickY_Hand);
    leftBumper = ps4.getRawButton(leftBumper_Port);
    rightBumper = ps4.getRawButton(rightBumper_Port);
    crossButton = ps4.getRawButton(crossButton_Port);
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

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
    }

    if (rightBumper) {
      intake.set(intake_Sensitivity);
    } 

    if (rightStickY != 0) {
      carry.set(rightStickY * -carry_Sensitivity);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
