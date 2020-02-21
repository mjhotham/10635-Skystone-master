package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.RevBulkData;


// @Autonomous
public class SlippageTest extends LinearOpMode {     // Current Red Auto

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;

    DriveConstraints gottaGoFast = new DriveConstraints(50, 30, 0, 2, .8, 0);
    DriveConstraints gottaGoSlow = new DriveConstraints(10, 10, 0, 2, .8, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.LeftIntake);

        lift.start(0);
        lift.stop();


        Pose2d startPos = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPos);
        Pose2d endPos = new Pose2d(24, -24, 3*Math.PI / 2);

        for (int i = 0; i < 10; i++) {
            drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).splineTo(endPos).build());
            sleep(1000);
            drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).reverse().splineTo(startPos).build());
        }
    }
}
