package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized2;

// @Autonomous
public class parkingAutonomous extends LinearOpMode {     //works but replaced with more user friendly version
    RobotConfig2 robot = new RobotConfig2();
    SampleMecanumDriveREVOptimized2 drive;

    int color = 1;
    int side = 1;
    int waitTime = 0;

    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized2(hardwareMap);

        while (!isStarted()) { //use this for switching between red and blue sides

            if (isStopRequested())
                return;

            if (gamepad1.a)
                color = 1;

            if (gamepad1.b)
                color = -1;

            if (gamepad1.x)
                side = 1;

            if (gamepad1.y)
                side = -1;

            if (gamepad1.dpad_up)
                waitTime += 1;

            if (gamepad1.dpad_down)
                waitTime -= 1;




            telemetry.addData("color", color == 1 ? "red" : "blue");
            telemetry.addData("side", side == 1 ? "left" : "right");
            telemetry.addData("Wait Time", waitTime);
            telemetry.update();

            sleep(100);
        }


        waitForStart();
        if (isStopRequested())
            return;



//        if (color == 1) {
//            drive.setPoseEstimate(new Pose2d(-63.5, -39, 0));
//        } else {
//           drive.setPoseEstimate(new Pose2d(63.5, -39, 0));
//        }
        sleep(waitTime * 1000);
        drive.setPoseEstimate(new Pose2d((-63.5 * color), -39, (.5 * Math.PI) - (.5 * Math.PI * color)));
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d((color * -48) - (12 * side), 0, Math.PI/2)).build());



    }
}