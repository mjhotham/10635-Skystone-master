package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

// @Autonomous
public class redSecondaryAutonomous extends LinearOpMode {   //Deprecated and non-functioning
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

        drive.setPoseEstimate(new Pose2d(-63.5, -39, 0));
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-60, 0, Math.PI / 2)).splineTo(new Pose2d(-63.5 + 16, 39)).build());

//        drive.setPoseEstimate(new Pose2d(-63.5, 39, 0));
//        drive.followTrajectorySync(drive.trajectoryBuilder().forward(16).build());

//  scan for stone order
        Position order = vuforia.getPosition(-19.8, -13, -2.8);//get numbers using getVuforiaNumbers teleop
        vuforia.stopVuforia();

        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Gripper.setPosition(drive.GripperOpen);
        switch (order) {
            case Left:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 0 + blockWidth * 3, -0.55)).forward(6).build());
                break;
            case Center:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 1 + blockWidth * 3, -0.55)).forward(6).build());
                break;
            case Right:
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 2 + blockWidth * 3, -0.55)).forward(6).build());
                break;
            default:
//                sleep(2000);//remove this later
                drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-31, 52 - blockWidth * 2  + blockWidth * 3, -0.55)).forward(6).build());
        }
//        drive.followTrajectorySync(drive.trajectoryBuilder().forward(6).build());
        drive.Gripper.setPosition(drive.GripperClosed);
        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(true).splineTo(new Pose2d(-60, 60, 0)).build());
        drive.turnSync(-Math.PI / 2);
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(-60, -4, Math.PI * 2.5)).build());


//        drive.turnSync(Math.PI / 2);

// https://www.youtube.com/watch?v=4UzU684dgZE


    }
}
