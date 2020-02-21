package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.RevBulkData;


// @Autonomous
public class AutobotsRollout extends LinearOpMode {     // what the hell is this opmode?

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;

    DriveConstraints gottaGoFast = new DriveConstraints(50, 30, 0, Math.PI, Math.PI, 0);

    Pose2d startPosition = new Pose2d(64, 36, Math.PI);//edit all of these positions at nicoles, headings should be correct

    Pose2d secondRightPickup = new Pose2d(33, 60, 3 * Math.PI / 2);
    Pose2d secondCenterPickup = new Pose2d(33, secondRightPickup.getY() - 8, 3 * Math.PI / 2);
    Pose2d secondLeftPickup = new Pose2d(33, secondCenterPickup.getY() - 8, 3 * Math.PI / 2);

    Vector2d firstRightPickup = new Vector2d(33, secondLeftPickup.getY() - 8);
    Vector2d firstCenterPickup = new Vector2d(33, firstRightPickup.getY() - 8);
    Vector2d firstLeftPickup = new Vector2d(33, firstCenterPickup.getY() - 8);

    Pose2d underBridge = new Pose2d(40, -10, 3 * Math.PI / 2);           //lift can move up here so don't break it, won't be used for a position anymore

    Pose2d quickDeposit = new Pose2d(33, -47, 3 * Math.PI / 2);
    Vector2d getWhateverItIsCalled = new Vector2d(30, -47);          // the foundation

//    void turnOnIntake() {
//        drive.LeftIntake.setPower(1);
//        drive.RightIntake.setPower(1);
//        drive.Gripper.setPosition(RobotConstants.GripperOpenAuto);
//        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntakeBlue);
//        drive.RightAngle.setPosition(RobotConstants.RightAngleIntakeBlue);
//    }
//
//    void turnOffIntake() {
//        drive.Gripper.setPosition(RobotConstants.GripperClosedAuto);
//        sleep(200);
//        drive.LeftIntake.setPower(0);
//        drive.RightIntake.setPower(0);
//        drive.LeftAngle.setPosition(RobotConstants.LeftAngleGrippedAuto);
//        drive.RightAngle.setPosition(RobotConstants.RightAngleGrippedAuto);
//    }

    void slideGrabberLeft() {

        sleep(1000);

    }

    void slideGrabberRight() {

        sleep(1000);

    }

    void grabberDown() {

        sleep(200);
    }

    void grabberDropPos() {//probably vertical preferably allows for travel near the bridge

    }

    void grabberBack() {

    }

    void grabberGrab() {

        sleep(200);
    }

    void grabberUngrab() {

    }

    @Override
    public void runOpMode() throws InterruptedException {


        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.LeftIntake);

        lift.start(0);
        lift.stop();

        drive.LeftAngle.setPosition(RobotConstants.LeftAngleScanningBlue);
        Position pos = scanAndWaitForStart.runOpMode(this); //includes wait for start stuff
        if (isStopRequested())
            return;

        drive.setPoseEstimate(startPosition);

        Vector2d firstStonePosition;
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


        //optimal get first stone option
        drive.followTrajectorySync(drive.trajectoryBuilder().lineTo(firstStonePosition,
                new LinearInterpolator(drive.getPoseEstimate().getHeading(),
                        Math.PI / 2 - drive.getPoseEstimate().getHeading()))
                .build()); //go pickup stone

        //option if it runs into the blocks the first time due to rotation happening too slowly
//        Vector2d intermediateFirstStonePosition = new Vector2d(
//                0.8 * firstStonePosition.getX() + 0.2 * drive.getPoseEstimate().getX(),
//                0.8 * firstStonePosition.getY() + 0.2 * drive.getPoseEstimate().getY()
//        );
//        drive.followTrajectorySync(drive.trajectoryBuilder().lineTo(intermediateFirstStonePosition,
//                new LinearInterpolator(drive.getPoseEstimate().getHeading(),
//                        Math.PI / 2 - drive.getPoseEstimate().getHeading()))
//                .lineTo(firstStonePosition, new ConstantInterpolator(Math.PI / 2))
//                .build()); //go pickup stone

        drive.update();
        grabberGrab();
        grabberBack();
        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).splineTo(quickDeposit).build());
        grabberDropPos();
        grabberUngrab();


        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).reverse().splineTo(secondStonePosition).build());
        grabberDown();
        grabberGrab();
        grabberBack();
        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).splineTo(quickDeposit).build());
        grabberDropPos();
        grabberUngrab();


        Pose2d thirdStonePosition = new Pose2d(firstStonePosition.getX(), firstStonePosition.getY() + 8, secondStonePosition.getHeading());
        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).reverse().splineTo(thirdStonePosition).build());
        grabberDown();
        grabberGrab();
        grabberBack();
        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).splineTo(quickDeposit).build());
        grabberDropPos();
        grabberUngrab();


        Pose2d fourthStonePosition = new Pose2d(thirdStonePosition.getX(), thirdStonePosition.getY() + 8, thirdStonePosition.getHeading());
        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).reverse().splineTo(fourthStonePosition).build());
        grabberDown();
        grabberGrab();
        grabberBack();
        drive.followTrajectorySync(drive.trajectoryBuilder(gottaGoFast).splineTo(quickDeposit).build());
        grabberDropPos();
        grabberUngrab();


        //go from grabber drop position to move foundation position
        drive.followTrajectory(drive.trajectoryBuilder()
                .strafeLeft(10)
                .lineTo(getWhateverItIsCalled,
                        new LinearInterpolator(drive.getPoseEstimate().getHeading(),
                                0 - drive.getPoseEstimate().getHeading()))
                .build());

        drive.LeftHook.setPosition(RobotConstants.LeftHookEngagedAuto);
        drive.RightHook.setPosition(RobotConstants.RightHookEngagedAuto);
        sleep(500);
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

        drive.LeftHook.setPosition(RobotConstants.LeftHookDisengagedAuto);
        drive.RightHook.setPosition(RobotConstants.RightHookDisengagedAuto);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(15).forward(44).build());

    }
}
