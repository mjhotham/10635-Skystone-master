package org.firstinspires.ftc.teamcode.State;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConfig2;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized2;

@Autonomous(name = "Parking Autonomous")
public class parkingAutonomous2 extends LinearOpMode {
    RobotConfig2 robot = new RobotConfig2();
    SampleMecanumDriveREVOptimized2 drive;

    int color = 1;
    int side = 1;
    int waitTime = 0;

    boolean PreviousDpadDown = false;
    boolean PreviousDpadUp = false;

    public void runOpMode() {

        telemetry.addData(">","Initializing");
        telemetry.update();

        drive = new SampleMecanumDriveREVOptimized2(hardwareMap);

        while (!isStarted()) {  //use this for switching between red and blue sides

            if (isStopRequested())
                return;

            if (gamepad1.a)   // Red
                color = 1;

            if (gamepad1.b)   // Blue
                color = -1;

            if (gamepad1.x)   // Inside
                side = 1;

            if (gamepad1.y)   // Outside
                side = -1;

            if (gamepad1.dpad_up) {
                if (!PreviousDpadUp) {
                    waitTime += 1;
                    PreviousDpadUp = true;
                }
            } else {
                PreviousDpadUp = false;
            }


            if (gamepad1.dpad_down) {
                if(!PreviousDpadDown) {
                    waitTime -= 1;
                    PreviousDpadDown = true;
                }
            } else {
                PreviousDpadDown = false;
            }

            waitTime = Range.clip(waitTime,0,25);

            telemetry.addData(">", "Red Alliance = A");
            telemetry.addData(">", "Blue Alliance = B");
            telemetry.addData(">", "Inside (near the center of the field) = X");
            telemetry.addData(">", "Outside (near the wall) = Y");

            telemetry.addLine();   //this should create an empty line to improve readability

            telemetry.addData("Selected Alliance Color", color == 1 ? "Red" : "Blue");
            telemetry.addData("Selected Side", side == 1 ? "Inside" : "Outside");
            telemetry.addData("Wait Time in Seconds", waitTime);
            telemetry.update();

        }


        waitForStart();
        if (isStopRequested())
            return;

        sleep(waitTime * 1000);
        drive.setPoseEstimate(new Pose2d((-63.5 * color), -39, (.5 * Math.PI) - (.5 * Math.PI * color)));
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d( color * (-48 + (12 * side)), 0, Math.PI/2)).build());

    }
}

