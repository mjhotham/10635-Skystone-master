package org.firstinspires.ftc.teamcode.State;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.RevBulkData;

@Autonomous(name = "Red 2x2 Base Auto")
public class StateRedSidebySideAuto extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;


    Pose2d startPosition = new Pose2d(-64, 36, 0);

    Pose2d firstLeftPickup = new Pose2d(-36, 58, .545);
    Pose2d firstCenterPickup = new Pose2d(-36, 48.5, .56);
    Pose2d firstRightPickup = new Pose2d(-36, 40.5, .58);

    Pose2d secondLeftPickup = new Pose2d(-36, 35, .55);
    Pose2d secondCenterPickup = new Pose2d(-36, 27, .56);
    Pose2d secondRightPickup = new Pose2d(-36, 18, .56);

    Pose2d underBridge = new Pose2d(-40, -10, Math.PI / 2);           //lift can move up here so don't break it

    Pose2d firstDeposit = new Pose2d(-37, -48.6, Math.PI - .45);
    Pose2d secondDeposit = new Pose2d(firstDeposit.getX(), firstDeposit.getY() + 4, firstDeposit.getHeading());

    Pose2d getWhateverItIsCalled = new Pose2d(-30, -47 - 8, Math.PI);          // the foundation


    void turnOnIntake() {
        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Gripper.setPosition(RobotConstants.GripperOpenAuto);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntakeRed);
        drive.RightAngle.setPosition(RobotConstants.RightAngleIntakeRed);


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
    public void runOpMode() throws InterruptedException {

        telemetry.addData(">", "Initializing");
        telemetry.update();

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.LeftIntake);
        lift.reset();
        telemetry.addData("LiftManager Target Height IN", () -> lift.liftTargetIN);
        telemetry.addData("LiftManager Current Height", () -> lift.LiftPositionIN);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleScanningRed);
        drive.setPoseEstimate(startPosition);

        Trajectory leftTrajectory = drive.trajectoryBuilder().splineTo(firstLeftPickup).forward(10).build();
        Trajectory centerTrajectory = drive.trajectoryBuilder().splineTo(firstCenterPickup).forward(10).build();
        Trajectory rightTrajectory = drive.trajectoryBuilder().splineTo(firstRightPickup).forward(10).build();

        Position pos = scanAndWaitForStart.runOpMode(this);
        if (isStopRequested())
            return;


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
         //go pickup stone
        switch (pos) {
            case Left:
                drive.followTrajectorySync(leftTrajectory);
                break;
            case Center:
                drive.followTrajectorySync(centerTrajectory);
                break;
            case Right:
                drive.followTrajectorySync(rightTrajectory);
                break;
            default:
                drive.followTrajectorySync(centerTrajectory);
                break;
        }
        drive.update();
        turnOffIntake(); //grab

        drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(underBridge).splineTo(firstDeposit).build());//all in reverse

        lift.slideTargetIN = 20.5;

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
                if (lift.SlidePositionIN > 10)//10? need speed right?
                    drive.Wrist.setPosition(.88);
            }
            drive.update();
        }

        drive.update();

        lift.start(2.75);
        while (lift.isBusy) {
            lift.update(drive.hub2.getBulkInputData());
            telemetry.update();
        }

        drive.Gripper.setPosition(drive.GripperOpen);

        lift.start(10);
        while (lift.isBusy) {
            lift.update(drive.hub2.getBulkInputData());
            telemetry.update();
        }

        drive.Wrist.setPosition(RobotConstants.WristCollectionPositionAuto);

        turnOnIntake();
        lift.liftTargetIN = 0;
        lift.slideTargetIN = 0;
        drive.followTrajectory(drive.trajectoryBuilder().splineTo(underBridge).splineTo(secondStonePosition).forward(10).build());
        while (drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.update();
        }

        turnOffIntake();//grab

        drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(underBridge).splineTo(secondDeposit).build());//all in reverse

        lift.slideTargetIN = 20.5;

        lift.start(RobotConstants.FirstDepositHeight);

        //deposit second skystone
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
                if (lift.SlidePositionIN > 10)//10? need speed right?
                    drive.Wrist.setPosition(.88);
            }
            drive.update();
        }

        drive.update();

        lift.start(2.75);
        while (lift.isBusy)
            lift.update(drive.hub2.getBulkInputData());
        drive.Gripper.setPosition(drive.GripperOpen);
//        lift.start(10);
//        while (lift.isBusy)
//            lift.update(drive.hub2.getBulkInputData());

        //foundation alignment
        drive.Wrist.setPosition(drive.WristCollectionPosition);
        lift.liftTargetIN = 0;
        lift.slideTargetIN = 0;
        drive.followTrajectory(drive.trajectoryBuilder().lineTo(new Vector2d(getWhateverItIsCalled.getX(), getWhateverItIsCalled.getY()), new LinearInterpolator(drive.getPoseEstimate().getHeading(), getWhateverItIsCalled.getHeading() - drive.getPoseEstimate().getHeading())).build());
        while (drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.update();
        }

        drive.LeftHook.setPosition(RobotConstants.LeftHookEngagedAuto);
        drive.RightHook.setPosition(RobotConstants.RightHookEngagedAuto);
        sleep(500);
        drive.followTrajectory(drive.trajectoryBuilder().strafeRight(2).forward(30).build());

        lift.liftTargetIN = 10;
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

        drive.setMotorPowers(1, 1, -1, -1);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntake);
        while (drive.getPoseEstimate().getHeading() > Math.PI / 2) {
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

        lift.stop();
        drive.LeftHook.setPosition(RobotConstants.LeftHookDisengagedAuto);
        drive.RightHook.setPosition(RobotConstants.RightHookDisengagedAuto);
        drive.LeftAngle.setPosition(RobotConstants.RightAngleTapePositionAuto);
        drive.followTrajectory(drive.trajectoryBuilder().back(15).lineTo(new Vector2d(drive.getPoseEstimate().getX() + 20, drive.getPoseEstimate().getY() + 10), new SplineInterpolator(Math.PI / 2, (Math.PI / 2) - .1)).build());

        drive.LeftAngle.setPosition(RobotConstants.RightAngleTapePositionAuto);
        lift.slideTargetIN = 0;
        lift.start(0);

        //run tape measure on delay
        new java.util.Timer().schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        drive.Tape.setPosition(RobotConstants.TapeExtendPower);
                        cancel();
                    }
                },
                1000
        );//feel free to use this other places because it's awesome

        while (drive.isBusy()) {
            RevBulkData bulkData = drive.hub2.getBulkInputData();
            if (bulkData != null) {
                lift.update(bulkData);
            }
            drive.LeftAngle.setPosition(RobotConstants.RightAngleTapePositionAuto);
            drive.update();
            telemetry.update();
        }

        sleep(1500);

//        drive.turnSync(Math.toRadians(-10));

//        while (getRuntime() < 30)
//            telemetry.update();

        //todo: remove reset code
//        //reset code
//        drive.Tape.setPosition(RobotConstants.TapeRetractPower);
//        drive.Wrist.setPosition(0.12);
//        lift.liftTargetIN = 0;
//        lift.slideTargetIN = 0;
//        drive.followTrajectory(drive.trajectoryBuilder().lineTo(new Vector2d(startPosition.getX(), startPosition.getY()), new LinearInterpolator(drive.getPoseEstimate().getHeading(), startPosition.getHeading() - drive.getPoseEstimate().getHeading())).build());
//        while (drive.isBusy()) {
//            RevBulkData bulkData = drive.hub2.getBulkInputData();
//            if (bulkData != null) {
//                lift.update(bulkData);
//            }
//            drive.update();
//        }
    }
}