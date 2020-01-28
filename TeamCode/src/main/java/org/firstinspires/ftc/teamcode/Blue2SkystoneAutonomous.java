package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized2;

// @Autonomous
public class Blue2SkystoneAutonomous extends LinearOpMode {        //Deprecated and non-functioning

    RobotConfig2 robot = new RobotConfig2();

    final double blockWidth = 8;

    double LiftTarget;
    double Time;

    private vuforiaLib vuforia = new vuforiaLib();
    SampleMecanumDriveREVOptimized2 drive;


    public void runOpMode() {

        vuforia.init(this);
        drive = new SampleMecanumDriveREVOptimized2(hardwareMap);

        robot.init(hardwareMap);

        vuforia.start();
        waitForStart();
        if (isStopRequested())
            return;

        drive.setPoseEstimate(new Pose2d(63.5, 39, Math.PI));               // Tell the program where the robot is starting on the field

//        drive.followTrajectorySync(drive.trajectoryBuilder().forward(16).build());      // drive forward for scanning (Hopefully UnNeeded)

        //  scan for stone order

        Position order = vuforia.getPosition(-19.8, -13, -2.8);       //get numbers using getVuforiaNumbers teleop         // probably need changing
        vuforia.stopVuforia();

        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Wrist.setPosition(robot.WristCollectionPosition);
        drive.LeftHook.setPosition(robot.LeftHookDisengaged);
        drive.RightHook.setPosition(robot.RightHookDisengaged);
        drive.Gripper.setPosition(robot.GripperOpen);

        switch (order) {
            case Left:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 0, 0.55 + Math.PI)).forward(6).build());    //  needs to be changed for far stones
                break;
            case Center:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 1, 0.55 + Math.PI)).forward(6).build());    //  needs to be changed for far stones
                break;
            case Right:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 2, 0.55 + Math.PI)).forward(6).build());    //  needs to be changed for far stones
                break;
            default:
                sleep(2000);  //remove this later
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 2 , 0.55 + Math.PI)).forward(6).build());   //  needs to be changed for far stones
        }

        drive.Gripper.setPosition(robot.GripperClosed);

        drive.followTrajectorySync(drive.trajectoryBuilder().back(13).setReversed(true).splineTo(new Pose2d(40, -10, Math.PI / 2)).build());   //drive back to inside of under bridge area to avoid teammate   (need to be changed)

        drive.LeftIntake.setPower(0);
        drive.RightIntake.setPower(0);

        drive.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftTarget = (1.1 * robot.LiftTicksPerInch * robot.MinimumTopSlideMovementHeightIN);

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        drive.LeftLift.setPower(1);
        drive.RightLift.setPower(1);

        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(true).splineTo(new Pose2d(30, -46, Math.PI)).build());    // drive to platform


        while ((robot.LeftLift.getCurrentPosition() * robot.LiftTicksPerInch) < robot.MinimumTopSlideMovementHeightIN) {      // wait for lift to get to minimum movement height
            drive.update();
        }

        robot.TopSlide.setPosition(robot.TopSlideExtendPower);

        while (robot.LeftIntake.getCurrentPosition() <= (10 * robot.TopSlideTicksPerInch)){              //  wait for topslide to leave robot before moving wrist
            drive.update();
        }

        drive.Wrist.setPosition(robot.WristNormalDepositPosition);

        while (robot.LeftIntake.getCurrentPosition() <= (robot.TopSlideExtendedPositionIN * robot.TopSlideTicksPerInch)){       // wait for top slide to get to delivery position
            drive.update();
        }

        drive.TopSlide.setPosition(robot.TopSlideOffPower);

        LiftTarget = (robot.FirstStoneHeight * robot.LiftTicksPerInch);      // set the new targets lift to FirstStoneHeight

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        while (robot.LeftLift.getCurrentPosition() >= LiftTarget ){    // wait for lift to lower to FirstStoneHeight
            drive.update();
        }

        Time = System.currentTimeMillis();

        while (System.currentTimeMillis() <= (time + 200) ){     // wait for things to settle down before opening gripper
            drive.update();
        }

        drive.Gripper.setPosition(robot.GripperOpen);

        Time = System.currentTimeMillis();

        while (System.currentTimeMillis() <= (time + 200) ){    // wait before lifting the lift after opening gripper
            drive.update();
        }

        LiftTarget = (robot.MinimumTopSlideMovementHeightIN * robot.LiftTicksPerInch);  // set lift target to min topslide movement height

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        while (robot.LeftLift.getCurrentPosition() <= LiftTarget){    // wait for lift to get to min topslide movement height
            drive.update();
        }

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(40, -10, Math.PI / 2)).build());

        drive.TopSlide.setPosition(robot.TopSlideRetractPower);

        drive.Wrist.setPosition(robot.WristCollectionPosition);

        while (robot.LeftIntake.getCurrentPosition() >= (.5 * robot.TopSlideTicksPerInch)){  // wait for topslide to move to the front of the robot
            drive.update();
        }

        robot.TopSlide.setPosition(robot.TopSlideOffPower);

        LiftTarget = 0;

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        while (robot.LeftLift.getCurrentPosition() > 0){    // wait for topslide to go down before going under bridge
            drive.update();
        }

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(0, 0, Math.PI / 2)).build());        // go towards second stone    (needs real values)

        robot.LeftIntake.setPower(1);                        // turn intake back on
        robot.RightIntake.setPower(1);


        switch (order) {
            case Left:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 0, -0.55)).forward(6).build());    //  needs to be changed for far stones
                break;
            case Center:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 1, -0.55)).forward(6).build());    //  needs to be changed for far stones
                break;
            case Right:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 2, -0.55)).forward(6).build());    //  needs to be changed for far stones
                break;
            default:
                sleep(2000);  //remove this later
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 2 , -0.55)).forward(6).build());   //  needs to be changed for far stones
        }


        robot.Gripper.setPosition(robot.GripperClosed);

        drive.followTrajectorySync(drive.trajectoryBuilder().back(10).setReversed(true).splineTo(new Pose2d(0, 0, Math.PI / 2)).build());      // back away from stone and go under bridge

        LiftTarget = (robot.SecondStoneHeight + 3) * robot.LiftTicksPerInch;

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        while (robot.LeftLift.getCurrentPosition() <= (robot.MinimumTopSlideMovementHeightIN * robot.LiftTicksPerInch)){    // wait for Lift to get to minimum topslide movement height
            drive.update();
        }

        robot.TopSlide.setPosition(robot.TopSlideExtendPower);

        while (robot.LeftIntake.getCurrentPosition() <= (robot.TopSlideExtendedPositionIN * robot.TopSlideTicksPerInch)){       // wait for top slide to get to delivery position
            drive.update();
        }

        robot.Wrist.setPosition(robot.WristNormalDepositPosition);

        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(true).splineTo(new Pose2d(27, -46, Math.PI)).build());    // drive to platform  to hook on and drop stone

        drive.LeftHook.setPosition(robot.LeftHookEngaged);
        drive.RightHook.setPosition(robot.RightHookEngaged);

        LiftTarget = (robot.SecondStoneHeight) * robot.LiftTicksPerInch;

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        while (robot.LeftLift.getCurrentPosition() >= LiftTarget ){    // wait for lift to lower to SecondStoneHeight
            drive.update();
        }

        Time = System.currentTimeMillis();

        while (System.currentTimeMillis() <= (time + 200) ){     // wait for things to settle down before opening gripper
            drive.update();
        }

        robot.Gripper.setPosition(robot.GripperOpen);

        Time = System.currentTimeMillis();

        while (System.currentTimeMillis() <= (time + 200) ){    // wait before lifting the lift after opening gripper
            drive.update();
        }

        LiftTarget = (robot.SecondStoneHeight + 3) * robot.LiftTicksPerInch;

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        while (robot.LeftLift.getCurrentPosition() <= (LiftTarget * robot.LiftTicksPerInch)){
            drive.update();
        }

        robot.Wrist.setPosition(robot.WristCollectionPosition);

        drive.followTrajectory(drive.trajectoryBuilder().forward(30).build());

        drive.waitForIdle();

        drive.setMotorPowers(1,1,-1,-1);

        while (drive.getPoseEstimate().getHeading() > Math.PI / 2) {
            drive.updatePoseEstimate();

        }

        drive.setMotorPowers(0, 0, 0, 0);

        drive.followTrajectorySync(drive.trajectoryBuilder().back(20).build());

        robot.TopSlide.setPosition(robot.TopSlideRetractPower);

        LiftTarget = (robot.MinimumTopSlideMovementHeightIN * robot.LiftTicksPerInch);

        drive.LeftLift.setTargetPosition((int) LiftTarget);
        drive.RightLift.setTargetPosition((int) LiftTarget);

        drive.followTrajectorySync(drive.trajectoryBuilder().forward(44).build());

        while (robot.LeftIntake.getCurrentPosition() <= 0 ){
            drive.update();

        }
    }
}
