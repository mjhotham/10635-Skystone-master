package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized2;

// @Autonomous
public class Red1SkystoneAutonomous extends LinearOpMode {    //Deprecated and non-functioning

    RobotConfig2 robot = new RobotConfig2();

    final double blockWidth = 8;
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

        drive.setPoseEstimate(new Pose2d(-63.5, 39, 0));  // Tell the program where the robot is starting on the field

        drive.followTrajectorySync(drive.trajectoryBuilder().forward(16).build());  // drive forward for scanning

        //  scan for stone order

        Position order = vuforia.getPosition(-19.8, -13, -2.8);    //get numbers using getVuforiaNumbers teleop
        vuforia.stopVuforia();


        drive.LeftIntake.setPower(1);        // turn on the intake motors
        drive.RightIntake.setPower(1);
        drive.Wrist.setPosition(robot.WristCollectionPosition);

        drive.Gripper.setPosition(robot.GripperOpen);

        drive.LeftHook.setPosition(robot.LeftHookDisengaged);
        drive.RightHook.setPosition(robot.RightHookDisengaged);

        switch (order) {
            case Left:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 0, -0.55)).forward(6).build());
                break;
            case Center:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 1, -0.55)).forward(6).build());
                break;
            case Right:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 2, -0.55)).forward(6).build());
                break;
            default:
                sleep(2000);  //remove this later
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 2 , -0.55)).forward(6).build());
        }




        drive.followTrajectorySync(drive.trajectoryBuilder().back(13).setReversed(true).splineTo(new Pose2d(-40, -10, Math.PI / 2)).build());

        drive.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.LeftLift.setTargetPosition((int) (1.2 * robot.LiftTicksPerInch * robot.MinimumTopSlideMovementHeightIN));
        drive.RightLift.setTargetPosition((int) (1.2 * robot.LiftTicksPerInch * robot.MinimumTopSlideMovementHeightIN));

        drive.LeftLift.setPower(1);
        drive.RightLift.setPower(1);

        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(true).splineTo(new Pose2d(-27, -46, Math.PI)).build());

        drive.LeftHook.setPosition(robot.LeftHookEngaged);
        drive.RightHook.setPosition(robot.RightHookEngaged);

        drive.followTrajectory(drive.trajectoryBuilder().forward(30).build());

        ElapsedTime TopSlideTime = new ElapsedTime();
        TopSlideTime.reset();

        drive.TopSlide.setPosition(robot.TopSlideExtendPower);

        while (TopSlideTime.seconds() < 1.5)
            drive.update();

        drive.Wrist.setPosition(robot.WristNormalDepositPosition);

        while (TopSlideTime.seconds() < 2)
            drive.update();

        drive.TopSlide.setPosition(0.5);

        drive.Gripper.setPosition(robot.GripperOpen);

//        while (TopSlideTime.seconds() < 3)
//            drive.update();
//        drive.TopSlide.setPosition(drive.TopSlideRetract);

        drive.waitForIdle();

        drive.setMotorPowers(1,1,-1,-1);

        while (drive.getPoseEstimate().getHeading() > Math.PI / 2) {
            drive.updatePoseEstimate();
//
        }

        drive.setMotorPowers(0, 0, 0, 0);

        drive.Wrist.setPosition(robot.WristCollectionPosition);
        drive.TopSlide.setPosition(0.5);
        drive.LeftLift.setTargetPosition((int) (16 * robot.LiftTicksPerInch));
        drive.RightLift.setTargetPosition((int) (16 * robot.LiftTicksPerInch));
        drive.LeftLift.setPower(1);
        drive.RightLift.setPower(1);


        drive.LeftHook.setPosition(robot.LeftHookDisengaged);
        drive.RightHook.setPosition(robot.RightHookDisengaged);

        drive.followTrajectorySync(drive.trajectoryBuilder().back(20)/*.build());
        drive.followTrajectorySync(drive.trajectoryBuilder()*/.forward(44).build());


//        drive.turnSync(Math.PI / 2);

// https://www.youtube.com/watch?v=4UzU684dgZE


//        Math.toRadians()


    }
}
