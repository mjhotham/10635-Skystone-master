package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class notSure extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;


    void openIntake() {
        drive.LeftIntake.setPower(0);
        drive.RightIntake.setPower(0);
        drive.Gripper.setPosition(RobotConstants.GripperOpen);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntake);
        drive.RightAngle.setPosition(RobotConstants.RightAngleIntake);
    }

    void turnOnIntake() {
        drive.LeftIntake.setPower(1);
        drive.RightIntake.setPower(1);
        drive.Gripper.setPosition(RobotConstants.GripperOpen);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntake);
        drive.RightAngle.setPosition(RobotConstants.RightAngleIntake);
    }

    void gripIntake() {
        drive.LeftIntake.setPower(0);
        drive.RightIntake.setPower(0);
        drive.Gripper.setPosition(RobotConstants.GripperClosed);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleGripped);
        drive.RightAngle.setPosition(RobotConstants.RightAngleGripped);
    }

    void ungripWheels() {
        drive.LeftIntake.setPower(0);
        drive.RightIntake.setPower(0);
        drive.Gripper.setPosition(RobotConstants.GripperClosed);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleOpen);
        drive.RightAngle.setPosition(RobotConstants.RightAngleOpen);
    }

    void ejectIntake() {
        ejectRequest = true;
        ejectTimer.reset();
        drive.Gripper.setPosition(RobotConstants.GripperOpen);
        drive.LeftAngle.setPosition(RobotConstants.LeftAngleIntake);
        drive.RightAngle.setPosition(RobotConstants.RightAngleIntake);

    }

    void intakeState(int intakeState) {
        switch (intakeState) {
            case -1:
                ejectIntake();
                break;
            case 0:
                openIntake();
                break;
            case 1:
                turnOnIntake();
                break;
            case 2:
                gripIntake();
                break;
            case 3:
                ungripWheels();
                break;
        }
    }

    int intakeState = 0;

    boolean previousRightBumper = false;
    boolean previousLeftBumper = false;
    boolean PreviousGamePad1B = false;

    boolean slideAtZero = false;

    double forward, right, spin;
    boolean previousLeftStickButton = false, previousRightStickButton = false, slowMode = false, reverseDrivetrain = false;
    boolean previousDpadUp = false, hooksEngaged = false;

    double wristRequestedPosition = -1;
    boolean wristCollectionRequest = false;

    ElapsedTime ejectTimer = new ElapsedTime();

    boolean ejectRequest = false;

    Servo CapStoneLift;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.LeftIntake);
        intakeState(intakeState);

        DigitalChannel zeroSwitch = hardwareMap.digitalChannel.get("ZeroSwitch");
        zeroSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("intakeState", () -> intakeState);
        telemetry.addData("lift.slideTargetIN", () -> lift.slideTargetIN);
        telemetry.addData("lift.liftTargetIN", () -> lift.liftTargetIN);
        telemetry.addData("lift.LiftPositionIN", () -> lift.LiftPositionIN);
        telemetry.addData("lift.SlidePositionIN", () -> lift.SlidePositionIN);
        telemetry.addData("lift.slideObstruction", () -> lift.slideObstruction);
        telemetry.addData("lift.liftObstruction", () -> lift.liftObstruction);
        telemetry.addData("wristCollectionRequest", () -> wristCollectionRequest);
        telemetry.addData("wristRequestedPosition", () -> wristRequestedPosition);

        CapStoneLift = hardwareMap.get(Servo.class, "CapStoneLift");

        waitForStart();
        while (opModeIsActive()) {
            RevBulkData bulkData2 = drive.hub2.getBulkInputData();

            if (intakeState != -1) {
                ejectRequest = false;
            }

            if (ejectRequest && ejectTimer.seconds() > .5) {
                ejectRequest = false;
                drive.LeftIntake.setPower(RobotConstants.OutTakePower);
                drive.RightIntake.setPower(RobotConstants.OutTakePower);
            }

            double triggerSum = gamepad1.right_trigger - gamepad1.left_trigger;
            if (triggerSum > 0.1 && intakeState == 2) {
                intakeState = 3;
                intakeState(intakeState);
            }

            if (gamepad2.right_bumper) {
                drive.Elbow.setPosition(Range.scale(gamepad2.left_stick_y, -1, 1, .2, .8));
                lift.update(bulkData2, triggerSum, gamepad1.back, true);
            } else
                lift.update(bulkData2, triggerSum, gamepad1.back, false);

            if (lift.slideTargetIN < 3 && lift.liftTargetIN < 6) {//code only active while slide and lift really close

                if (gamepad1.right_bumper) {
                    if (!previousRightBumper) {
                        intakeState = intakeState == 1 ? 2 : 1;//intake or wheel gripped
                        drive.Wrist.setPosition(RobotConstants.WristCollectionPosition);
                        intakeState(intakeState);
                    }
                    previousRightBumper = true;
                } else {
                    previousRightBumper = false;
                }

                if (gamepad1.left_bumper) {
                    if (!previousLeftBumper) {
                        intakeState = intakeState == -1 ? 0 : -1;//open or outake
                        intakeState(intakeState);
                    }
                    previousLeftBumper = true;
                } else {
                    previousLeftBumper = false;
                }

            } else {

                if (gamepad1.left_bumper) {
                    if (!previousLeftBumper) {
                        previousLeftBumper = true;
                        drive.LeftIntake.setPower(RobotConstants.OutTakePower);
                        drive.RightIntake.setPower(RobotConstants.OutTakePower);
                    }
                } else if (previousLeftBumper) {
                    previousLeftBumper = false;
                    drive.LeftIntake.setPower(0);
                    drive.RightIntake.setPower(0);
                }

                if (gamepad1.right_bumper) {
                    if (!previousRightBumper) {
                        intakeState = intakeState == 3 ? 0 : 3;
                        intakeState(intakeState);
                    }
                    previousRightBumper = true;
                } else {
                    previousRightBumper = false;
                }
            }

            if (gamepad1.dpad_down) {
                intakeState = 0;
                intakeState(intakeState);
                lift.slideTargetIN = 0;
                lift.liftTargetIN = 0;
                wristCollectionRequest = true;
            }

            if (gamepad1.x) {
                wristRequestedPosition = RobotConstants.WristRightDepositPosition;
                lift.slideTargetIN = RobotConstants.XTopslideTarget;
                if (lift.liftTargetIN < 8) {
                    lift.liftTargetIN = 8;
                }
                drive.Gripper.setPosition(RobotConstants.GripperClosed);
                intakeState = 3;
                intakeState(intakeState);
            }

            if (gamepad1.y) {
                wristRequestedPosition = RobotConstants.WristFrontDepositPosition;
                lift.slideTargetIN = RobotConstants.YTopslideTarget;
                if (lift.liftTargetIN < 8) {
                    lift.liftTargetIN = 8;
                }
                drive.Gripper.setPosition(RobotConstants.GripperClosed);
                intakeState = 3;
                intakeState(intakeState);
            }


            if (gamepad1.b) {                 // Toggle Gripper no matter the superstructures state even though gamepad1.right_bumper is probably sufficient because of the conditions
                if (!PreviousGamePad1B) {

                    if (drive.Gripper.getPosition() != RobotConstants.GripperOpen) {
                        drive.Gripper.setPosition(RobotConstants.GripperOpen);

                    } else if (drive.Gripper.getPosition() != RobotConstants.GripperClosed) {
                        drive.Gripper.setPosition(RobotConstants.GripperClosed);
                    }

                    PreviousGamePad1B = true;
                }
            } else
                PreviousGamePad1B = false;


            if (wristRequestedPosition != -1 && lift.SlidePositionIN > 12) {//only use for positions requiring extension
                drive.Wrist.setPosition(wristRequestedPosition);
                wristRequestedPosition = -1;
            } else if (wristCollectionRequest && lift.SlidePositionIN < 12) {//only use for going to collection position
                drive.Wrist.setPosition(RobotConstants.WristCollectionPosition);
                wristCollectionRequest = false;
            }

            if (gamepad1.dpad_up) {
                if (!previousDpadUp) {
                    previousDpadUp = true;
                    hooksEngaged = !hooksEngaged;
                    drive.LeftHook.setPosition(hooksEngaged ? RobotConstants.LeftHookEngaged : RobotConstants.LeftHookDisengaged);
                    drive.RightHook.setPosition(hooksEngaged ? RobotConstants.RightHookEngaged : RobotConstants.RightHookDisengaged);
                }
            } else
                previousDpadUp = false;


            if (gamepad1.left_stick_button) {                       // mason can decide if he likes this mapping
                if (!previousLeftStickButton) {
                    previousLeftStickButton = true;
                    slowMode = !slowMode;
                }
            } else
                previousLeftStickButton = false;


            if (gamepad1.right_stick_button) {                      // mason can decide if he likes this mapping
                if (!previousRightStickButton) {
                    previousRightStickButton = true;
                    reverseDrivetrain = !reverseDrivetrain;
                }
            } else
                previousRightStickButton = false;

            forward = gamepad1.left_stick_y * (reverseDrivetrain ? 1 : -1) * (slowMode ? 0.3 : 1);
            right = gamepad1.left_stick_x * (reverseDrivetrain ? -1 : 1) * (slowMode ? 0.3 : 1);
            spin = gamepad1.right_stick_x * (slowMode ? 0.4 : 1);

            double maxwheel = Math.abs(forward) + Math.abs(right) + Math.abs(spin);
            if (maxwheel > 1) {
                forward /= maxwheel;
                right /= maxwheel;
                spin /= maxwheel;
            }

            drive.setMotorPowers(forward + spin + right, forward + spin - right, forward - spin + right, forward - spin - right);

            boolean slideSensor = !bulkData2.getDigitalInputState(zeroSwitch);//this is reversed, deal with it
            if (!slideAtZero && slideSensor) {
                drive.Elbow.setPosition(0.5);
                drive.LeftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeState(intakeState);
                slideAtZero = true;
            }
            if (slideAtZero && lift.SlidePositionIN > 4 && !slideSensor)
                slideAtZero = false;
            telemetry.addData("slideAtZero", slideAtZero);

            CapStoneLift.setPosition(Range.scale(gamepad2.right_stick_y, -1.0, 1.0, .2, .8));

            telemetry.update();
        }
    }
}
