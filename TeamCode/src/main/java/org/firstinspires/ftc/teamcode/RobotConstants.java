package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class RobotConstants {

  // This file contains most of the constants that may need t be adjusted between now and the competition.

   // All target values are in inches


    // For Auto and TELEOP

  public static final double TapeExtendPower = .8;

  public static final double TapeRetractPower = .2;

  public static final double LiftMotorTicksPerRotationofOuputShaft = 386.3;

//  public static final double TopSlideMinMovementHeight = 8; // Placeholder Value

    // For TELEOP


  public static final double WristCollectionPosition = .12;            // Dpad_Down
  public static final double WristFrontDepositPosition = .787;         // Y
  public static final double WristRightDepositPosition = .44;          // X

  public static final double LeftHookDisengaged = .42;
  public static final double LeftHookEngaged = .12;

  public static final double RightHookDisengaged = .32;
  public static final double RightHookEngaged = .62;


  public static final double LeftAngleOpen = Range.scale(1211, 500, 2500, 0, 1);
  public static final double LeftAngleOuttake = Range.scale(1170, 500, 2500, 0, 1);
  public static final double LeftAngleIntake = Range.scale(1150, 500, 2500, 0, 1);
  public static final double LeftAngleGripped = Range.scale(1102, 500, 2500, 0, 1);


  public static final double RightAngleOpen = Range.scale(1544, 500, 2500, 0, 1);
  public static final double RightAngleOuttake = Range.scale(1560, 500, 2500, 0, 1);
  public static final double RightAngleIntake = Range.scale(1580, 500, 2500, 0, 1);
  public static final double RightAngleGripped = Range.scale(1653, 500, 2500, 0, 1);

  public static final double RightAngleTapePosition = Range.scale(1460, 500, 2500, 0, 1);          // makes the tape stick out as straight as possible

  public static final double GripperOpen = .87;
  public static final double GripperClosed = .5;

  public static final double WristOverRideSpeed = .002;

  public static final double YTopSlideTarget = 15.8;                  // Y
  public static final double XTopSlideTarget = 15.6;                  // X

  public static final double TopSlideCapstonePickupPosition = 9.5;

//  public static final double LiftCapstonePickupHeight = 8;  // placeholder value

  public static final double OutTakePower = -.18;                        // may need to change this to avoid launching penalties

  public static final double TeleInakePower = .5;

  public static final double TeleHoldingPower = .05;


  // used fot lean correction code

  public static final double[] LeanLiftPositions = {0, 14.404, 17.8, 21.439, 25.57, 28.6, 32.4, 35.9, 38.9, 41.9};   // lift heights in inches

  public static final double[] LeanTopSlidePositions = {16, 16.768, 16.8467, 17.0955, 17.2656, 17.6, 17.318, 17.488, 18.064, 18.28};   // TopSlide positions in inches



    // For BOTH Autos


  public static final double FirstDepositTopSlideTarget = 16;         // make sure the first stone is completely on the foundation so the robot doesn't push it when it comes back with the second stone
  public static final double SecondDepositTopSlideTarget = 15;

  public static final double FirstDepositHeight = 8;
  public static final double SecondDepositHeight = 13;                // lower will cause the stone to bounce less but could collide with first stone





         // these are unlikely to be different from the TeleOp values but they are here just in case they need adjustment specific to auto

  public static final double GripperOpenAuto = .87;
  public static final double GripperClosedAuto = .5;

  public static final double WristCollectionPositionAuto = .12;
  public static final double WristDepositPositionAuto = .767;

  public static final double LeftHookDisengagedAuto = .42;
  public static final double LeftHookEngagedAuto = .12;

  public static final double RightHookDisengagedAuto = .32;
  public static final double RightHookEngagedAuto = .62;

  public static final double LeftAngleGrippedAuto = Range.scale(1122, 500, 2500, 0, 1);
  public static final double LeftAngleOpenAuto = Range.scale(1211, 500, 2500, 0, 1);

  public static final double RightAngleOpenAuto = Range.scale(1544, 500, 2500, 0, 1);
  public static final double RightAngleGrippedAuto = Range.scale(1633, 500, 2500, 0, 1);
  public static final double RightAngleTapePositionAuto = Range.scale(1460, 500, 2500, 0, 1);          // makes the tape stick out as straight as possible



  // For RED Auto


  public static final double LeftAngleScanningRed = Range.scale(1275, 500, 2500, 0, 1);      // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static final double LeftAngleIntakeRed = Range.scale(1150, 500, 2500, 0, 1);
  public static final double RightAngleIntakeRed = Range.scale(1580, 500, 2500, 0, 1);



    // For Blue Auto


  public static final double LeftAngleScanningBlue = Range.scale(1260, 500, 2500, 0, 1);     // used in init to aim the camera at the 3 outer stones

  // would only change these to improve the intaking of the skystones

  public static final double LeftAngleIntakeBlue = Range.scale(1150, 500, 2500, 0, 1);
  public static final double RightAngleIntakeBlue = Range.scale(1580, 500, 2500, 0, 1);


}
