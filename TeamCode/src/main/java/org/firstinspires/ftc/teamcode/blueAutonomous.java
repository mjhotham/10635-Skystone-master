package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

// @Autonomous
public class blueAutonomous extends LinearOpMode {    // deprecated
    RobotConfig robot = new RobotConfig();
    final double blockWidth = 8;
    private vuforiaLib vuforia = new vuforiaLib();
    SampleMecanumDriveREVOptimized drive;

    int color = 1;

    public void runOpMode() {
        vuforia.init(this);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        while (!isStarted()) { //use this for switching between red and blue sides
            if (isStopRequested())
                return;

//            if (gamepad1.a)
//                color = 1;
//            if (gamepad1.b)
//                color = -1;
//
//            telemetry.addData("color", color == 1 ? "red" : "blue");
            telemetry.update();
        }

        vuforia.start();
        waitForStart();
        if (isStopRequested())
            return;

        drive.setPoseEstimate(new Pose2d(63.5, 39, Math.PI));
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(16).build());

//  scan for stone order
        Position order = vuforia.getPosition(-19.8, -13, -2.8);//get numbers using getVuforiaNumbers teleop
        vuforia.stopVuforia();
// drive forward to make room for strafing
//        justMove(-27, 0, 0);
// strafe towards chosen skystone
        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Gripper.setPosition(drive.GripperOpen);
        switch (order) {
            case Left:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 2, 0.55 + Math.PI)).forward(6).build());
                break;
            case Center:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 1, 0.55 + Math.PI)).forward(6).build());
                break;
            case Right:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 0, 0.55 + Math.PI)).forward(6).build());
                break;
            default:
                sleep(2000);//remove this later
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(31, 52 - blockWidth * 0 * Math.random(), 0.55 + Math.PI)).forward(6).build());
        }
//        drive.followTrajectorySync(drive.trajectoryBuilder().forward(6).build());
        drive.Gripper.setPosition(drive.GripperClosed);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(13).setReversed(true).splineTo(new Pose2d(40, -10, Math.PI / 2)).build());
        drive.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.LeftLift.setTargetPosition((int) (1.2 * drive.LiftTicksPerInch * drive.MinimumElbowMovementHeightIN));
        drive.RightLift.setTargetPosition((int) (1.2 * drive.LiftTicksPerInch * drive.MinimumElbowMovementHeightIN));
        drive.LeftLift.setPower(1);
        drive.RightLift.setPower(1);
        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(true).splineTo(new Pose2d(27, -46, 0)).build());

        drive.LeftHook.setPosition(drive.LeftHookEngaged);
        drive.RightHook.setPosition(drive.RightHookEngaged);

        drive.followTrajectory(drive.trajectoryBuilder().forward(30).build());
        ElapsedTime elbowTime = new ElapsedTime();
        elbowTime.reset();
        drive.Elbow.setPosition(drive.ElbowExtend);
        while (elbowTime.seconds() < 1.5)
            drive.update();
        drive.Wrist.setPosition(drive.WristFrontDepositPosition);
        while (elbowTime.seconds() < 2)
            drive.update();
        drive.Elbow.setPosition(0.5);
        drive.Gripper.setPosition(robot.GripperOpen);
//        while (elbowTime.seconds() < 3)
//            drive.update();
//        drive.Elbow.setPosition(drive.ElbowRetract);

        drive.waitForIdle();

        drive.setMotorPowers(-1,-1,1,1);

        while (drive.getPoseEstimate().getHeading() < Math.PI / 2 || drive.getPoseEstimate().getHeading() > Math.PI) {
            drive.updatePoseEstimate();
//            if (elbowTime.seconds() > 3) {
//                drive.Elbow.setPosition(0.5);
//                drive.LeftLift.setTargetPosition(0);
//                drive.RightLift.setTargetPosition(0);
//                drive.LeftLift.setPower(.3);
//                drive.RightLift.setPower(.3);
//                elbowTime.reset();
//            }
        }
        drive.setMotorPowers(0, 0, 0, 0);

        drive.Wrist.setPosition(drive.WristCollectionPosition);
        drive.Elbow.setPosition(0.5);
        drive.LeftLift.setTargetPosition((int) (16 * drive.LiftTicksPerInch));
        drive.RightLift.setTargetPosition((int) (16 * drive.LiftTicksPerInch));
        drive.LeftLift.setPower(1);
        drive.RightLift.setPower(1);


        drive.LeftHook.setPosition(drive.LeftHookDisengaged);
        drive.RightHook.setPosition(drive.RightHookDisengaged);

        drive.followTrajectorySync(drive.trajectoryBuilder().back(20)/*.build());
        drive.followTrajectorySync(drive.trajectoryBuilder()*/.forward(44).build());


//        drive.turnSync(Math.PI / 2);

// https://www.youtube.com/watch?v=4UzU684dgZE


    }
}
