/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;


  //Noah's playground code
    
  public static int A_button = 1;
  public static int JoystickPort = 0;
  //drive train
  public static int LB = 12;
  public static int RB = 15;
  public static int RF = 17;
  public static int LF = 13;
  //////Sensors/////
  //Encoders
  public static int EncoderLeft = 12;
  public static int EncoderRight = 15;

  //PigeonController
  public static int PigImu = 17;
  //Gyro
  public static int GyroID = 0;
  //RangeFinder
  public static int RangeID = 0;

  

}
