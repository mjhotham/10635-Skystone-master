package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.RevBulkData;


// @Autonomous
public class bestAutonomousblue extends LinearOpMode {     //Deprecated and non-functioning

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;

    Pose2d startPosition = new Pose2d(64,36, Math.PI);

    Pose2d firstLeftPickup = new Pose2d(36,40.5,Math.PI - .56);
    Pose2d firstCenterPickup = new Pose2d(36,48.5, Math.PI - .56);
    Pose2d firstRightPickup = new Pose2d(36,58 ,Math.PI - .545);

    Pose2d secondLeftPickup = new Pose2d(36,35,Math.PI - .56);
    Pose2d secondCenterPickup = new Pose2d(36,27,Math.PI - .56);
    Pose2d secondRightPickup = new Pose2d(36,18,Math.PI - .56);

    Pose2d underBridge = new Pose2d(40, -10, Math.PI / 2);           //lift can move up here so don't break it

    Pose2d quickDeposit = new Pose2d(33,-47,0);
    Pose2d getWhateverItIsCalled = new Pose2d(30,-47,0);  // the red or blue bumpy thingy

    void turnOnIntake() {
        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Gripper.setPosition(drive.GripperOpen);
        drive.LeftAngle.setPosition(drive.LeftAngleIntake);
        drive.RightAngle.setPosition(drive.RightAngleIntake);
    }

    void turnOffIntake() {    // try setting angles to gripped? I will test to see whats most reliable
        drive.Gripper.setPosition(drive.GripperClosed);
        sleep(1000);
        drive.LeftIntake.setPower(0);
        drive.RightIntake.setPower(0);
    }

    boolean LiftStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {


        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.LeftIntake);

        drive.setPoseEstimate(startPosition);

        Position pos = Position.Unknown;

        Pose2d firstStonePosition;
        switch (pos) {
            case Left:
                firstStonePosition = firstLeftPickup;
                break;
            case Center:
                firstStonePosition = firstCenterPickup;
                break;
            case Right:
                firstStonePosition = firstRightPickup;
                break;
            default:
                firstStonePosition = firstCenterPickup;
                break;
        }

        Pose2d secondStonePosition;
        switch (pos) {
            case Left:
                secondStonePosition = secondLeftPickup;
                break;
            case Center:
                secondStonePosition = secondCenterPickup;
                break;
            case Right:
                secondStonePosition = secondRightPickup;
                break;
            default:
                secondStonePosition = secondCenterPickup;
                break;
        }

        turnOnIntake();
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(firstStonePosition).forward(8).build()); //go pickup stone
        drive.update();
        turnOffIntake(); //grab

        drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(underBridge).splineTo(quickDeposit).build());//all in reverse

        lift.slideTargetIN = 14;  // the number you are looking for is 14

        lift.start(7);
        //deposit first skystone
        while (lift.isBusy || drive.isBusy()) {
            if (drive.getPoseEstimate().getY() < underBridge.getY()) {//only run code after passed bridge
                RevBulkData bulkData = drive.hub2.getBulkInputData();
                drive.LeftAngle.setPosition(drive.LeftAngleOpen);
                drive.RightAngle.setPosition(drive.RightAngleOpen);
                telemetry.addData("LeftLift Encoder", bulkData.getMotorCurrentPosition(drive.LeftLift));
                telemetry.addData("RightLift Encoder", bulkData.getMotorCurrentPosition(drive.RightLift));
                telemetry.addData("LiftManager Target Height IN", lift.liftTargetIN);
                telemetry.addData("LiftManager Current Height", lift.LiftPositionIN);
                telemetry.update();

                if (LiftStarted = false){
                    LiftStarted = true;
                }

                lift.update(bulkData);//updates lift and slide checking for collisions
                if (lift.SlidePositionIN > 12)
                    drive.Wrist.setPosition(drive.WristFrontDepositPosition);
            }
            drive.update();
        }
        lift.stop();

        drive.update();
        drive.Gripper.setPosition(drive.GripperOpen);//drop first skystone
        sleep(100);
        drive.Wrist.setPosition(drive.WristCollectionPosition);

        lift.liftTargetIN = 0;
        lift.slideTargetIN = 0;
        drive.followTrajectory(drive.trajectoryBuilder().splineTo(underBridge).build());
        while (lift.isBusy || drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.update();
        }
        turnOnIntake();
        //make sure this part works because if it doesn't the next part will kill it
        //there's only so much code I'm willing to test at once

        //select lines and Ctrl+/ to comment/uncomment lines without wasting time

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(secondStonePosition).forward(8).build());
        turnOffIntake();//grab

        drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(underBridge).splineTo(getWhateverItIsCalled).build());//all in reverse

        lift.start(12);
        lift.slideTargetIN = 14;  // the number you are looking for is 14
        //deposit second skystone
        while (lift.isBusy || drive.isBusy()) {
            if (drive.getPoseEstimate().getY() < underBridge.getY()) {//only run code after passed bridge
                RevBulkData bulkData = drive.hub2.getBulkInputData();
        drive.LeftAngle.setPosition(drive.LeftAngleOpen);
        drive.RightAngle.setPosition(drive.RightAngleOpen);
                if (bulkData != null) {
                    lift.update(bulkData);//updates lift and slide checking for collisions
                    if (lift.SlidePositionIN > 12)
                        drive.Wrist.setPosition(drive.WristFrontDepositPosition);
                }
            }
            drive.update();
        }
        drive.Gripper.setPosition(drive.GripperOpen);
        drive.LeftHook.setPosition(drive.LeftHookEngaged);
        drive.RightHook.setPosition(drive.RightHookEngaged);
        sleep(500);
        drive.Wrist.setPosition(drive.WristCollectionPosition);
        drive.followTrajectory(drive.trajectoryBuilder().forward(30).build());
        lift.liftTargetIN = 0;
        lift.slideTargetIN = 0;
        while (drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.update();
        }

        drive.setMotorPowers(-1, -1, 1, 1);
        while (drive.getPoseEstimate().getHeading() < Math.PI / 2 || drive.getPoseEstimate().getHeading() > Math.PI) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.updatePoseEstimate();
        }
        drive.setMotorPowers(0, 0, 0, 0);

//        check up to here before adding the last part
        lift.stop();
        drive.LeftHook.setPosition(drive.LeftHookDisengaged);
        drive.RightHook.setPosition(drive.RightHookDisengaged);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(15).forward(44).build());


    }
}
