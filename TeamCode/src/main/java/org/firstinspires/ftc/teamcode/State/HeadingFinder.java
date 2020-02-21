package org.firstinspires.ftc.teamcode.State;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

// @Autonomous(name="Heading Finder")
public class HeadingFinder extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;

    Pose2d startPosition = new Pose2d(-64, 36, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        drive.setPoseEstimate(startPosition);

        telemetry.addData("Heading In Radians", ()-> drive.getExternalHeading());

        while(opModeIsActive()){
            drive.update();
            telemetry.update();
        }

    }
}