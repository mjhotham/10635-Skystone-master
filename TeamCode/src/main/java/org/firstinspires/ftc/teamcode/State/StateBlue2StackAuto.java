package org.firstinspires.ftc.teamcode.State;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.RevBulkData;

@Autonomous(name = "Blue 2 Stone Stacking Auto")
public class StateBlue2StackAuto extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;

    Pose2d startPosition = new Pose2d(64, 36, Math.PI);

    Pose2d firstLeftPickup = new Pose2d(36, 40.5, Math.PI - .58);
    Pose2d firstCenterPickup = new Pose2d(36, 48.5, Math.PI - .56);
    Pose2d firstRightPickup = new Pose2d(36, 58, Math.PI - .545);

    Pose2d secondLeftPickup = new Pose2d(36, 18, Math.PI - .56);
    Pose2d secondCenterPickup = new Pose2d(36, 27, Math.PI - .56);
    Pose2d secondRightPickup = new Pose2d(36, 35, Math.PI - .55);

    Pose2d underBridge = new Pose2d(40, -10, Math.PI / 2);           //lift can move up here so don't break it

    Pose2d quickDeposit = new Pose2d(33, -47, 0);
    Pose2d getWhateverItIsCalled = new Pose2d(33, -47, 0);          // the foundation

    void turnOnIntake() {
        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Gripper.setPosition(RobotConstants.GripperOpenAuto);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntakeBlue);
        drive.RightAngle.setPosition(RobotConstants.RightAngleIntakeBlue);


    }

    void turnOffIntake() {
        drive.Gripper.setPosition(RobotConstants.GripperClosedAuto);
        sleep(200);
        drive.LeftIntake.setPower(0);
        drive.RightIntake.setPower(0);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleGrippedAuto);
        drive.RightAngle.setPosition(RobotConstants.RightAngleGrippedAuto);
    }

    boolean LiftStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {      // Red stacking Auto?

        telemetry.addData(">", "Initializing");
        telemetry.update();

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.Elbow2, drive.LeftIntake);

        telemetry.addData("LiftManager Target Height IN", () -> lift.liftTargetIN);
        telemetry.addData("LiftManager Current Height", () -> lift.LiftPositionIN);

        drive.LeftAngle.setPosition(RobotConstants.LeftAngleScanningBlue);
        Position pos = scanAndWaitForStart.runOpMode(this);
        if (isStopRequested())
            return;

        drive.setPoseEstimate(startPosition);

        drive.Wrist.setPosition(RobotConstants.WristCollectionPositionAuto);

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
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(firstStonePosition).forward(10).build()); //go pickup first stone
        drive.update();
        turnOffIntake(); //grab

        drive.followTrajectory(drive.trajectoryBuilder().back(3).reverse().splineTo(underBridge).splineTo(quickDeposit).build());  //all in reverse

        lift.slideTargetIN = RobotConstants.FirstDepositTopSlideTarget;

        lift.start(RobotConstants.FirstDepositHeight);
        //deposit first skystone
        while (lift.isBusy || drive.isBusy()) {
            if (drive.getPoseEstimate().getY() < underBridge.getY()) {//only run code after passed bridge
                RevBulkData bulkData = drive.hub2.getBulkInputData();
                drive.LeftAngle.setPosition(RobotConstants.LeftAngleOpenAuto);
                drive.RightAngle.setPosition(RobotConstants.RightAngleOpenAuto);
                telemetry.addData("LeftLift Encoder", bulkData.getMotorCurrentPosition(drive.LeftLift));
                telemetry.addData("RightLift Encoder", bulkData.getMotorCurrentPosition(drive.RightLift));
                telemetry.update();

                if (LiftStarted = false) {
                    LiftStarted = true;
                }

                lift.update(bulkData);//updates lift and slide checking for collisions
                if (lift.SlidePositionIN > 12)
                    drive.Wrist.setPosition(RobotConstants.WristDepositPositionAuto);
            }
            drive.update();
        }

        drive.update();

        lift.start(2.75);
        while (lift.isBusy) {
            lift.update(drive.hub2.getBulkInputData());
            telemetry.update();
        }

        drive.Gripper.setPosition(RobotConstants.GripperOpenAuto);  //drop first skystone
        sleep(200);

        lift.start(2.75 + 6);
        while (lift.isBusy) {
            lift.update(drive.hub2.getBulkInputData());
            telemetry.update();
        }


        drive.Wrist.setPosition(RobotConstants.WristCollectionPositionAuto);

        turnOnIntake();
        lift.liftTargetIN = 0;
        lift.slideTargetIN = 0;
        drive.followTrajectory(drive.trajectoryBuilder().splineTo(underBridge).splineTo(secondStonePosition).forward(8).build());
        while (drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.update();
        }


        turnOffIntake();   //grab

        drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(underBridge).splineTo(getWhateverItIsCalled).forward(-3).build());   //all in reverse

        lift.start(RobotConstants.SecondDepositHeight);
        lift.slideTargetIN = RobotConstants.SecondDepositTopSlideTarget;  // the number you are looking for is 14


        //deposit second skystone
        while (lift.isBusy || drive.isBusy()) {
            if (drive.getPoseEstimate().getY() < underBridge.getY()) {//only run code after passed bridge
                RevBulkData bulkData = drive.hub2.getBulkInputData();
                drive.LeftAngle.setPosition(RobotConstants.LeftAngleOpenAuto);
                drive.RightAngle.setPosition(RobotConstants.RightAngleOpenAuto);
                if (bulkData != null) {
                    lift.update(bulkData);//updates lift and slide checking for collisions
                    if (lift.SlidePositionIN > 12)
                        drive.Wrist.setPosition(RobotConstants.WristDepositPositionAuto);
                }
            }
            drive.update();
            telemetry.update();
        }

        drive.LeftHook.setPosition(RobotConstants.LeftHookEngagedAuto);
        drive.RightHook.setPosition(RobotConstants.RightHookEngagedAuto);
        sleep(500);
        //todo: check the strafe direction
        drive.followTrajectory(drive.trajectoryBuilder().strafeRight(2).forward(30).build());

        lift.liftTargetIN = 6.6 - 4;
//        lift.slideTargetIN = 0;

        while (drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);

                if (lift.LiftPositionIN < 6.6 - 3 && lift.liftTargetIN == 6.6 - 4) {
                    lift.liftTargetIN = 6.6 + 10;
                    drive.Gripper.setPosition(RobotConstants.GripperOpenAuto);
                }
            }
            drive.update();
            telemetry.update();
        }

        drive.setMotorPowers(-1, -1, 1, 1);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntake);
        while (drive.getPoseEstimate().getHeading() < Math.PI / 2 || drive.getPoseEstimate().getHeading() > 3 * Math.PI / 2) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);

                if (lift.LiftPositionIN < 6.6 - 3 && lift.liftTargetIN == 6.6 - 4) {
                    lift.liftTargetIN = 6.6 + 10;
                    drive.Gripper.setPosition(RobotConstants.GripperOpenAuto);
                }
            }
            drive.updatePoseEstimate();
            telemetry.update();
        }
        drive.setMotorPowers(0, 0, 0, 0);

        drive.Tape.setPosition(RobotConstants.TapeExtendPower);
        lift.stop();
        drive.LeftHook.setPosition(RobotConstants.LeftHookDisengagedAuto);
        drive.RightHook.setPosition(RobotConstants.RightHookDisengagedAuto);
        drive.RightAngle.setPosition(RobotConstants.RightAngleTapePositionAuto);
        drive.followTrajectory(drive.trajectoryBuilder().back(15).forward(25).build());

        drive.Wrist.setPosition(RobotConstants.WristCollectionPosition);
        lift.slideTargetIN = 0;
        lift.start(0);
        while (drive.isBusy() || lift.isBusy) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.update();
            telemetry.update();
        }
/*

        drive.setMotorPowers(1, 1, -1, -1);
        while (drive.getPoseEstimate().getHeading() > ((Math.PI / 2) - .5)){
            drive.updatePoseEstimate();
            telemetry.update();
        }
        drive.setMotorPowers(0, 0, 0, 0);
*/

    }
}
