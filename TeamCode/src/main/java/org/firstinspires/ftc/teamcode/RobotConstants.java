package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

class RobotConstants {

   // This file contains most of the constants that may need to be adjusted between now and the competition.

   // All target values are in inches


    // For Auto and TELEOP

  public static double TapeExtendPower = .8;

  public static double TapeRetractPower = .2;

  public static double LiftMotorTicksPerRotationofOuputShaft = 386.3;

//  public static double TopSlideMinMovementHeight = 8; // Placeholder Value

    // For TELEOP


  public static double WristCollectionPosition = .11;            // Dpad_Up
  public static double WristFrontDepositPosition = .787;         // Y
  public static double WristRightDepositPosition = .43;          // X
    
  public static double LeftHookDisengaged = .42;
  public static double LeftHookEngaged = .12;

  public static double RightHookDisengaged = .32;
  public static double RightHookEngaged = .62;


  public static double LeftAngleOpen = Range.scale(1211,500,2500,0,1);
  public static double LeftAngleOuttake = Range.scale(1170,500,2500,0,1);
  public static double LeftAngleIntake = Range.scale(1150,500,2500,0,1);
  public static double LeftAngleGripped = Range.scale(1122,500,2500,0,1);


  public static double RightAngleOpen = Range.scale(1544,500,2500,0,1);
  public static double RightAngleOuttake = Range.scale(1560,500,2500,0,1);
  public static double RightAngleIntake = Range.scale(1580,500,2500,0,1);
  public static double RightAngleGripped = Range.scale(1633,500,2500,0,1);

  public static double RightAngleTapePosition = Range.scale(1460,500,2500,0,1);          // makes the tape stick out as straight as possible

  public static double GripperOpen = .87;
  public static double GripperClosed = .5;

  public static double WristOverRideSpeed = .002;

  public static double YTopSlideTarget = 15.8;                  // Y
  public static double XTopSlideTarget = 15.6;                  // X

  public static double TopSlideCapstonePickupPosition = 10.5;

//  public static double LiftCapstonePickupHeight = 8;  // placeholder value

  public static double OutTakePower = -.18;                        // may need to change this to avoid launching penalties

  public static double TeleInakePower = .5;

  public static double TeleHoldingPower = .05;


  // used fot lean correction code

  public static double[] LeanLiftPositions =     {0,  14.404, 17.8,    21.439,  25.57,   28.6,  32.4,   35.9,   38.9,   41.9};   // lift heights in inches

  public static double[] LeanTopSlidePositions = {16, 16.768, 16.8467, 17.0955, 17.2656, 17.6, 17.318, 17.488, 18.064, 18.28};   // TopSlide positions in inches





    // For BOTH Autos


  public static double FirstDepositTopSlideTarget = 16;         // make sure the first stone is completely on the foundation so the robot doesn't push it when it comes back with the second stone
  public static double SecondDepositTopSlideTarget = 16;

  public static double FirstDepositHeight = 8;
  public static double SecondDepositHeight = 13;                // lower will cause the stone to bounce less but could collide with first stone





         // these are unlikely to be different from the TeleOp values but they are here just in case they need adjustment specific to auto

  public static double GripperOpenAuto = .87;
  public static double GripperClosedAuto = .5;

  public static double WristCollectionPositionAuto = .11;           
  public static double WristDepositPositionAuto = .767;         
    
  public static double LeftHookDisengagedAuto = .42;
  public static double LeftHookEngagedAuto = .12;

  public static double RightHookDisengagedAuto = .32;
  public static double RightHookEngagedAuto = .62;

  public static double LeftAngleGrippedAuto = Range.scale(1122,500,2500,0,1);
  public static double LeftAngleOpenAuto = Range.scale(1211,500,2500,0,1);

  public static double RightAngleOpenAuto = Range.scale(1544,500,2500,0,1);
  public static double RightAngleGrippedAuto = Range.scale(1633,500,2500,0,1);
  public static double RightAngleTapePositionAuto = Range.scale(1460,500,2500,0,1);          // makes the tape stick out as straight as possible



  // For RED Auto


  public static double LeftAngleScanningRed = Range.scale(1211,500,2500,0,1);      // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static double LeftAngleIntakeRed = Range.scale(1150,500,2500,0,1);
  public static double RightAngleIntakeRed = Range.scale(1580,500,2500,0,1);



    // For Blue Auto


  public static double LeftAngleScanningBlue = Range.scale(1260,500,2500,0,1);     // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static double LeftAngleIntakeBlue = Range.scale(1150,500,2500,0,1);
  public static double RightAngleIntakeBlue = Range.scale(1580,500,2500,0,1);


}
