package org.firstinspires.ftc.teamcode;

class RobotConstants {

   // This file contains most of the constants that may need to be adjusted between now and the competition.

   // All target values are in inches


    // For TELEOP


  public static double WristCollectionPosition = .11;            // Dpad_Up
  public static double WristFrontDepositPosition = .787;         // Y
  public static double WristRightDepositPosition = .43;          // X
    
  public static double LeftHookDisengaged = .42;
  public static double LeftHookEngaged = .12;

  public static double RightHookDisengaged = .32;
  public static double RightHookEngaged = .62;

  public static double LeftAngleOpen = 0.3;
  public static double LeftAngleIntake = 0.353;
  public static double LeftAngleGripped = 0.42;

  public static double RightAngleOpen = 0.58;
  public static double RightAngleIntake = 0.52;
  public static double RightAngleGripped = 0.45;

  public static double GripperOpen = 1;
  public static double GripperClosed = .64;

  public static double WristOverRideSpeed = .002;

  public static double YTopSlideTarget = 15.6;                  // Y
  public static double XTopSlideTarget = 15.6;                  // X

  public static double TopSlideCapstonePickupPosition = 10.5;

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

  public static double LeftAngleGrippedAuto = 0.42;
  public static double LeftAngleOpenAuto = 0.3;

  public static double RightAngleOpenAuto = 0.58;
  public static double RightAngleGrippedAuto = 0.45;



    // For RED Auto


  public static double LeftAngleScanningRed = .28;      // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static double LeftAngleIntakeRed = 0.353;
  public static double RightAngleIntakeRed = 0.52;



    // For Blue Auto


  public static double LeftAngleScanningBlue = .27;    // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static double LeftAngleIntakeBlue = 0.353;
  public static double RightAngleIntakeBlue = 0.52;


}
