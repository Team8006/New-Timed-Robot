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

  private Spark frontLeft;
  private Spark rearLeft;
  private Spark frontRight;
  private Spark rearRight;
  private Spark intake;
  private Spark carry;

  private SpeedControllerGroup left;
  private SpeedControllerGroup right;
  private DifferentialDrive robotDrive;

  private static final int frontLeftPort = 8;
  private static final int rearLeftPort = 9;
  private static final int frontRightPort = 0;
  private static final int rearRightPort = 1;
  private static final int intakePort = 7;
  private static final int carryPort = 2;

  private Joystick ps4;
  private static final int ps4Port = 0;

  private final Timer timer = new Timer();
   
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Assigning Spark ID
    frontLeft = new Spark(frontLeftPort);
    rearLeft = new Spark(rearLeftPort);
    frontRight = new Spark(frontRightPort);
    rearRight = new Spark(rearRightPort);
    intake = new Spark(intakePort);
    carry = new Spark(carryPort);

    // Drivetrain
    left = new SpeedControllerGroup(frontLeft, rearLeft); 
    right = new SpeedControllerGroup(frontRight, rearRight);
    robotDrive = new DifferentialDrive(left, right);
    
    // Assigning PS4 Controller ID
    ps4 = new Joystick(ps4Port);
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
    CameraServer server = CameraServer.getInstance();
    server.startAutomaticCapture();
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    // Sensitivity
    double move_sensitivity = 0.4;
    double rotate_sensitivity = 0.5;

    // Movement
    double ps4_move = ((ps4.getRawAxis(4) + 0.5) - (ps4.getRawAxis(3) + 0.5)) * move_sensitivity;
    double ps4_rotate = ps4.getX(Hand.kLeft) * rotate_sensitivity;
    robotDrive.arcadeDrive(ps4_move, ps4_rotate);

    // Button Mapping
    if (ps4.getRawButton(2)) {
      rotate_sensitivity = 0.8;
    } else {
      rotate_sensitivity = 0.4;
    } 

    if (ps4.getRawButton(5)) {
      intake.set(-0.5);
    }

    if (ps4.getRawButton(6)) {
      intake.set(0.5);
    } 

    if (ps4.getRawAxis(5) != 0) {
      carry.set(ps4.getRawAxis(5) * -0.5);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
