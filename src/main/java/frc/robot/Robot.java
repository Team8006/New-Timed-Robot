/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.buttons.Button;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    /*SpeedController frontLeftBlow = new PWMVictorSPX(0);
    SpeedController rearLeftBlow = new PWMVictorSPX(1);
    SpeedControllerGroup left = new SpeedControllerGroup(frontLeftBlow, rearLeftBlow);
    
    SpeedController frontRightSuck = new PWMVictorSPX(2);
    SpeedController rearRightSuck = new PWMVictorSPX(3);
    SpeedControllerGroup right = new SpeedControllerGroup(frontRightSuck, rearRightSuck);

    //SpeedController leftBlow = new PWMVictorSPX(5);
    //SpeedController rightSuck = new PWMVictorSPX(6);
 
    DifferentialDrive robotDrive = new DifferentialDrive(left, right);*/

    Spark frontLeft = new Spark(8);
    Spark rearLeft = new Spark(9);
    Spark frontRight = new Spark(0);
    Spark rearRight = new Spark(1);
    Spark intake = new Spark(7);
    Spark carry = new Spark(2);
   
    SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft); 
    SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);
    DifferentialDrive robotDrive = new DifferentialDrive(left, right);
    
    private final Joystick ps4 = new Joystick(0);
    private final Timer timer = new Timer();

  // distance in inches the robot wants to stay from an object
  private static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  private static final double kValueToInches = 0.125;

  // proportional speed constant
  private static final double kP = 0.05;

  // ultrasonic
  private static final int ultrasonic_port = 0;
  private final AnalogInput m_ultrasonic = new AnalogInput(ultrasonic_port);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
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
      robotDrive.arcadeDrive(-0.5, 0.0); // drive forwards half speed
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

    double move_sensitivity = 0.4;
    double rotate_sensitivity = 0.5;
    double ps4_move = ((ps4.getRawAxis(4) + 0.5) - (ps4.getRawAxis(3) + 0.5)) * move_sensitivity;
    double ps4_rotate = ps4.getX(Hand.kLeft) * rotate_sensitivity;

    if (ps4.getRawButton(1)==true)
    {
      rotate_sensitivity = 1.0;
    }
    if (ps4.getRawButton(1)==false)
    {
      rotate_sensitivity = 0.5;
    } 

    if (ps4.getRawButton(5)) {
      intake.set(-0.5);
    } else if (ps4.getRawButton(5) == false) { 
      intake.stopMotor();
    }

    if (ps4.getRawButton(6)) {
      intake.set(0.5);
    } else if (ps4.getRawButton(6) == false) { 
      intake.stopMotor();
    }

    if (ps4.getRawAxis(5) != 0) {
      carry.set(ps4.getRawAxis(5) * -0.5);
      //rightBlow.set(oshit);
    } else {
      carry.stopMotor();
    }
    //rightBlow.set(oshit);
    robotDrive.arcadeDrive(ps4_move, ps4_rotate);

    // sensor returns a value from 0-4095 that is scaled to inches
    double currentDistance = m_ultrasonic.getValue() * kValueToInches;

    // convert distance error to a motor speed
    double currentSpeed = (kHoldDistance - currentDistance) * kP;

    // drive robot
    robotDrive.arcadeDrive(currentSpeed, 0);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
