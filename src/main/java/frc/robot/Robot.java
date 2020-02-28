/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.buttons.Button;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    SpeedController frontLeftBlow = new PWMVictorSPX(0);
    SpeedController rearLeftBlow = new PWMVictorSPX(1);
    SpeedControllerGroup left = new SpeedControllerGroup(frontLeftBlow, rearLeftBlow);
    
    SpeedController frontRightSuck = new PWMVictorSPX(2);
    SpeedController rearRightSuck = new PWMVictorSPX(3);
    SpeedControllerGroup right = new SpeedControllerGroup(frontRightSuck, rearRightSuck);

    //SpeedController leftBlow = new PWMVictorSPX(5);
    //SpeedController rightSuck = new PWMVictorSPX(6);
 
    DifferentialDrive robotDrive = new DifferentialDrive(left, right);

    /*CANSparkMax frontLeft = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax leftSuck = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax rightBlow = new CANSparkMax(5, MotorType.kBrushless);
   
    SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft); 
    SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);
    DifferentialDrive robotDrive = new DifferentialDrive(left, right);*/
    
    private final Joystick ps4 = new Joystick(0);
    private final Timer timer = new Timer();
   
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
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    double move_sensitivity = 0.5;
    double rotate_sensitivity = 0.70;

    if (ps4.getRawButton(1))
    {
      rotate_sensitivity = 1.0;
    }
    if (ps4.getRawButton(1))
    {
      rotate_sensitivity = 0.70;
    }

    if (ps4.getRawButton(5)) {
      left.set(-1);
    } else { 
      ;left.stopMotor();
    }

    if (ps4.getRawButton(6)) {
      right.set(1);
    } else {
      right.stopMotor();
    }

    double ps4_move = (ps4.getRawAxis(3) - ps4.getRawAxis(4)) * move_sensitivity;
    double ps4_rotate = ps4.getX(Hand.kLeft) * rotate_sensitivity; 
    robotDrive.arcadeDrive(ps4_move, ps4_rotate);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
