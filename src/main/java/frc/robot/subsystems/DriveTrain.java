/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Robot;
import frc.robot.commands.TankDriveWithJoystick;

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
public class DriveTrain extends Subsystem {

  //Setting up Talon Controllers. Make sure to use WPI_TalonSRX instead of TalonSRX
  private final WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(RobotMap.LF);
  private final WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(RobotMap.RF);
  private final WPI_TalonSRX m_backLeft = new WPI_TalonSRX(RobotMap.LB);
  private final WPI_TalonSRX m_backRight = new WPI_TalonSRX(RobotMap.RB);
  private final SpeedController example = new Spark(0);
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(m_leftMotor, m_backLeft);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(m_rightMotor, m_backRight);
  private final DifferentialDrive m_drive
      = new DifferentialDrive(leftMotors, rightMotors);

  private final Encoder m_RobotEncoder = new Encoder(RobotMap.EncoderLeft, RobotMap.EncoderRight);
  //private final AnalogInput m_rangefinder = new AnalogInput(RobotMap.RangeID);
  //private final AnalogGyro m_gyro = new AnalogGyro(RobotMap.GyroID);

  /**
   * Create a new drive train subsystem.
   */
  public DriveTrain() {
    super();

    // Encoders may measure differently in the real world and in
    // simulation. In this example the robot moves 0.042 barleycorns
    // per tick in the real world, but the simulated encoders
    // simulate 360 tick encoders. This if statement allows for the
    // real robot to handle this difference in devices.
    //ALL OF THIS IS REFERENCE FROM THE EXAMPLE. UNSPECIFIC TO OUR ROBOT
    /*if (Robot.isReal()) {
      m_leftEncoder.setDistancePerPulse(0.042);
      m_rightEncoder.setDistancePerPulse(0.042);
    } else {
      // Circumference in ft = 4in/12(in/ft)*PI
      m_leftEncoder.setDistancePerPulse((4.0 / 12.0 * Math.PI) / 360.0);
      m_rightEncoder.setDistancePerPulse((4.0 / 12.0 * Math.PI) / 360.0);
    }*/

    // Let's name the sensors on the LiveWindow
    addChild("Drive", m_drive);
    //addChild("Left Encoder", m_leftEncoder);From Example code
    addChild("Right Encoder", m_RobotEncoder);
    //addChild("Rangefinder", m_rangefinder);
    //addChild("Gyro", m_gyro);
  }

  /**
   * When no other command is running let the operator drive around using the
   * PS3 joystick.
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDriveWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {
    SmartDashboard.putNumber("Left Distance", m_RobotEncoder.getDistance());
    SmartDashboard.putNumber("Right Speed", m_RobotEncoder.getRate());
    //SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void drive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param joy The ps3 style joystick to use to drive tank style.
   */
  public void drive(Joystick joy) {
    drive(-joy.getY(), -joy.getThrottle());
  }

  public void drive(XboxController xbox)
  {
    drive(-xbox.getRawAxis(0), -xbox.getRawAxis(1));
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  //public double getHeading() {
    //return m_gyro.getAngle();
  //}

  /**
   * Reset the robots sensors to the zero states.
   */
  public void reset() {
    //m_gyro.reset();
    m_RobotEncoder.reset();
    //m_rightEncoder.reset();
  }

  /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  /*public double getDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
  }*/

  /**
   * Get the distance to the obstacle.
   *
   * @return The distance to the obstacle detected by the rangefinder.
   */
  //public double getDistanceToObstacle() {
    // Really meters in simulation since it's a rangefinder...
    //return m_rangefinder.getAverageVoltage();
  //}
}