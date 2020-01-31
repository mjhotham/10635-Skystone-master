package org.firstinspires.ftc.teamcode;

class RobotConstants {

   // This file contains most of the constants that may need to be adjusted between now and the competition.

   // All target values are in inches


    // For TELEOP


  public static double WristCollectionPosition = .11;            // Dpad_Up
  public static double WristFrontDepositPosition = .767;         // Y
  public static double WristRightDepositPosition = .43;          // X
    
  public static double LeftHookDisengaged = .42;
  public static double LeftHookEngaged = .12;

  public static double RightHookDisengaged = .32;
  public static double RightHookEngaged = .62;

  public static double LeftAngleOpen = 0.69833;
  public static double LeftAngleIntake = 0.71722;
  public static double LeftAngleGripped = 0.73;

  public static double RightAngleOpen = 0.85;
  public static double RightAngleIntake = 0.81;
  public static double RightAngleGripped = 0.77;

  public static double GripperOpen = 1;
  public static double GripperClosed = .64;

  public static double WristOverRideSpeed = .002;

  public static double YTopSlideTarget = 16.67;                  // Y
  public static double XTopSlideTarget = 16.67;                  // X

  public static double OutTakePower = -1;                        // may need to change this to avoid launching penalties



    // For BOTH Autos


  public static double FirstDepositTopSlideTarget = 16;         // make sure the first stone is completely on the foundation so the robot doesn't push it when it comes back with the second stone
  public static double SecondDepositTopSlideTarget = 14;

  public static double FirstDepositHeight = 7;
  public static double SecondDepositHeight = 12;                // lower will cause the stone to bounce less but could collide with first stone



         // these are unlikely to be different from the TeleOp values but they are here just in case they need adjustment specific to auto

  public static double GripperOpenAuto = 1;
  public static double GripperClosedAuto = .64;

  public static double WristCollectionPositionAuto = .11;           
  public static double WristDepositPositionAuto = .767;         
    
  public static double LeftHookDisengagedAuto = .42;
  public static double LeftHookEngagedAuto = .12;

  public static double RightHookDisengagedAuto = .32;
  public static double RightHookEngagedAuto = .62;

  public static double LeftAngleGrippedAuto = 0.73;
  public static double LeftAngleOpenAuto = 0.69833;

  public static double RightAngleOpenAuto = 0.85;
  public static double RightAngleGrippedAuto = 0.77;



    // For RED Auto


  public static double LeftAngleScanningRed = .67;      // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static double LeftAngleIntakeRed = 0.71722;
  public static double RightAngleIntakeRed = 0.81;




    // For Blue Auto


  public static double LeftAngleScanningBlue = .68;    // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static double LeftAngleIntakeBlue = 0.71722;
  public static double RightAngleIntakeBlue = 0.81;


}
