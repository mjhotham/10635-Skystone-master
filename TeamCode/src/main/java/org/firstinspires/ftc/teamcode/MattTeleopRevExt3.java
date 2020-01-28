package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.openftc.revextensions2.RevBulkData;


// @TeleOp
public class MattTeleopRevExt3 extends LinearOpMode {


    RobotConfig2 robot = new RobotConfig2();

    public RevBulkData bulkData, bulkData2;

    double forward;
    double right;
    double spin;

    double triggerSum;

    double TopSlideTargetIN;

    double liftPower = 0;

    double LoopFrequency;

    double TopSlideScalar;      // topslide power proportional to topslide distance to target

    int IntakePreviousLoopState = 0;              // 0 for off      1 for intake      2 for outtake

    int DesiredSuperStructureState = 0;           // 0 for manual mode (for lift)    1  for intake position     2 for regular deposit     3 for 90 degree deposit    4 for capstone pickup position

    int LiftEncoderDifference;

    int LiftTarget;

    int LoopCount = 0;

    long StartTime;

    boolean slowMode = false;

    boolean reverseDrivetrain = false;

    boolean previousDpadUp = false;

    boolean PreviousGamePad1B = false;

    boolean previousLeftStickButton = false;

    boolean previousRightStickButton = false;

    boolean LiftUnderManualControl = false;

    boolean Gampepad1RightBumperPreviousLoopState = false;

    boolean Gamepad1LeftBumperPreviousLoopState = false;

    ElapsedTime stallTimer = new ElapsedTime();

    double topSlidePositionOffset = 0;

    boolean liftRequest = false;
    boolean liftObstruction = false;
    boolean topSlideRequest = false;
    boolean wristObstruction = false;
    boolean wristRequest = false;
    double topSlideTargetIN = 0;
    double liftTargetIN = 0;
    double prevTopSlidePosition = -100;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        bulkData = robot.ExpansionHub1.getBulkInputData();
        bulkData2 = robot.ExpansionHub2.getBulkInputData();


        robot.Gripper.setPosition(robot.GripperOpen);
        robot.TopSlide.setPosition(.5);
        robot.Wrist.setPosition(robot.WristCollectionPosition);
        robot.LeftHook.setPosition(robot.LeftHookDisengaged);
        robot.RightHook.setPosition(robot.RightHookDisengaged);

        telemetry.addData("Say", "Initialization Complete");
        telemetry.addData("Gripper Position", () -> robot.Gripper.getPosition());
//        telemetry.addData("Wrist Position", () -> robot.Wrist.getPosition());
        telemetry.addData("TopSlide Power", () -> robot.TopSlide.getPosition());
        telemetry.addData("TopSlide encoder", () -> bulkData2.getMotorCurrentPosition(robot.LeftIntake));
        telemetry.addData("TopSlidePositionIN", () -> robot.TopSlidePositionIN);
        telemetry.addData("TopSlide RPM", () -> robot.TopSlideRPM);                                              // want to know what kind of load the 393 is under
        telemetry.addData("Lift Inches", () -> robot.LiftPositionIN);
        telemetry.addData("LeftLift Power", () -> robot.LeftLift.getPower());
        telemetry.addData("RightLift Power", () -> robot.RightLift.getPower());
        telemetry.addData("LeftLift Encoder Counts", () -> bulkData2.getMotorCurrentPosition(robot.LeftLift));
        telemetry.addData("RightLift Encoder Counts", () -> bulkData2.getMotorCurrentPosition(robot.RightLift));
        telemetry.addData("Lift Encoder Count Difference", () -> LiftEncoderDifference);
        telemetry.addData("left Lift Motor RPM", () -> robot.LeftLiftRPM);
        telemetry.addData("Right Lift Motor RPM", () -> robot.RightLiftRPM);
        telemetry.addData("Loop Frequency", () -> LoopFrequency);
        telemetry.addData("Desired SuperStructure State", () -> DesiredSuperStructureState);

        telemetry.addData("liftRequest", () -> liftRequest);
        telemetry.addData("wristRequest", () -> wristRequest);
        telemetry.addData("topSlideRequest", () -> topSlideRequest);
        telemetry.addData("liftObstruction", () -> liftObstruction);
        telemetry.addData("wristObstruction", () -> wristObstruction);

        telemetry.addData("liftTargetIN", () -> liftTargetIN);
        telemetry.addData("topSlideTargetIN", () -> topSlideTargetIN);


        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {


            if (LoopCount == 0) {
                StartTime = System.currentTimeMillis();
            }

            if (isStopRequested()) {
                return;
            }

            bulkData = robot.ExpansionHub1.getBulkInputData();      // not sure which motors are on which hub

            if (isStopRequested()) {
                return;
            }
            bulkData2 = robot.ExpansionHub2.getBulkInputData();

            if (isStopRequested()) {
                return;
            }

            robot.LiftPositionIN = (bulkData2.getMotorCurrentPosition(robot.LeftLift) + bulkData2.getMotorCurrentPosition(robot.RightLift)) / (2 * robot.LiftTicksPerInch);

            robot.TopSlidePositionIN = bulkData2.getMotorCurrentPosition(robot.LeftIntake) / robot.TopSlideTicksPerInch - topSlidePositionOffset;

            LiftEncoderDifference = Math.abs(bulkData2.getMotorCurrentPosition(robot.RightLift) - bulkData2.getMotorCurrentPosition(robot.LeftLift));

            robot.LeftLiftRPM = (bulkData2.getMotorVelocity(robot.LeftLift) * 60) / robot.LiftMotorTicksPerRotationofOuputShaft;            // assumes getMotorVelocity reports in ticks per second
            robot.RightLiftRPM = (bulkData2.getMotorVelocity(robot.RightLift) * 60) / robot.LiftMotorTicksPerRotationofOuputShaft;          // "                                                   "
            robot.TopSlideRPM = ((bulkData2.getMotorVelocity(robot.LeftIntake)) * 60) / robot.TopSlideTicksPerRoatationOfVexle;             // "                                                   "


//          DriveTrain Code

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


            forward = gamepad1.left_stick_y * (reverseDrivetrain ? 1 : -1) * (slowMode ? 0.4 : 1);
            right = gamepad1.left_stick_x * (reverseDrivetrain ? -1 : 1) * (slowMode ? 0.4 : 1);
            spin = gamepad1.right_stick_x * (slowMode ? 0.5 : 1);

            double maxwheel = Math.abs(forward) + Math.abs(right) + Math.abs(spin);
            if (maxwheel > 1) {
                forward /= maxwheel;
                right /= maxwheel;
                spin /= maxwheel;
            }

            robot.FrontLeft.setPower(forward + spin + right);
            robot.FrontRight.setPower(forward - spin - right);
            robot.BackLeft.setPower(forward + spin - right);
            robot.BackRight.setPower(forward - spin + right);

//          End DriveTrain Code


//          Hooks Code

            if (gamepad1.dpad_up) {
                if (!previousDpadUp) {
                    previousDpadUp = true;
                    robot.hooksEngaged = !robot.hooksEngaged;
                    robot.LeftHook.setPosition(robot.hooksEngaged ? robot.LeftHookEngaged : robot.LeftHookDisengaged);
                    robot.RightHook.setPosition(robot.hooksEngaged ? robot.RightHookEngaged : robot.RightHookDisengaged);
                }
            } else
                previousDpadUp = false;

//          End Hooks Code

//           Manual Lift Code

            triggerSum = gamepad1.right_trigger - gamepad1.left_trigger;

            if (Math.abs(triggerSum) > 0.1) {

                if (!LiftUnderManualControl) {
                    LiftUnderManualControl = true;
                    robot.LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                liftPower = Range.clip(triggerSum, robot.LiftPositionIN > 0 || gamepad1.back ? Range.clip(liftPower - 0.1, -1, 0) : 0, 1);                  //limit lift speed

                double liftOffset = (bulkData2.getMotorCurrentPosition(robot.LeftLift) - bulkData2.getMotorCurrentPosition(robot.RightLift)) / (robot.LeftLift.getMotorType().getTicksPerRev());

                robot.LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                robot.RightLift.setPower(liftPower + Math.max(0, liftOffset));


            } else if (LiftUnderManualControl) {
                int target = Math.max(bulkData2.getMotorCurrentPosition(robot.LeftLift), bulkData2.getMotorCurrentPosition(robot.RightLift));
                robot.LeftLift.setTargetPosition(target);
                robot.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LeftLift.setPower(robot.LiftPIDPower);
                robot.RightLift.setTargetPosition(target);
                robot.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RightLift.setPower(robot.LiftPIDPower);
                LiftUnderManualControl = false;


            } else if (gamepad1.back) {
                robot.LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            } else if (liftRequest) {

                LiftTarget = (int) Math.round((liftObstruction ? robot.MinimumTopSlideMovementHeightIN : liftTargetIN) * robot.LiftTicksPerInch);

                robot.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.LeftLift.setTargetPosition(LiftTarget);
                robot.RightLift.setTargetPosition(LiftTarget);

                liftPower = 1;

                double liftOffset = (bulkData2.getMotorCurrentPosition(robot.LeftLift) - bulkData2.getMotorCurrentPosition(robot.RightLift)) / (robot.LeftLift.getMotorType().getTicksPerRev());

                robot.LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                robot.RightLift.setPower(liftPower + Math.max(0, liftOffset));

                if (Math.abs(robot.LiftPositionIN - liftTargetIN) < 1) {
                    liftRequest = false;
                    robot.LeftLift.setPower(robot.LiftPIDPower);
                    robot.RightLift.setPower(robot.LiftPIDPower);
                }
            }


            if (gamepad1.dpad_left) {
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                robot.LeftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else
                robot.LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//          Gamepad State control COde

            if (gamepad1.dpad_down) {            //  return to ready to intake

                if (!LiftUnderManualControl) {           // Maybe Unnecessary
                    DesiredSuperStructureState = 1;

                    IntakePreviousLoopState = 0;
                    liftRequest = true;
                    topSlideRequest = true;
                    wristRequest = true;
                    liftTargetIN = 0;
                    topSlideTargetIN = robot.TopSlideIntakePositionIN;
                }
            }

            if (gamepad1.y) {                   // deliver with 180 spin

                DesiredSuperStructureState = 2;

                topSlideRequest = true;
                wristRequest = true;
                if (robot.LiftPositionIN < robot.MinimumTopSlideMovementHeightIN) {
                    liftRequest = true;
                    liftTargetIN = robot.MinimumTopSlideMovementHeightIN + 1;
                }
                topSlideTargetIN = robot.TopSlideExtendedPositionIN;
            }

            if (gamepad1.x) {                   //    deliver with 90 spin

                DesiredSuperStructureState = 3;

                topSlideRequest = true;
                wristRequest = true;
                if (robot.LiftPositionIN < robot.MinimumTopSlideMovementHeightIN) {
                    liftRequest = true;
                    liftTargetIN = robot.MinimumTopSlideMovementHeightIN + 1;
                }
                topSlideTargetIN = robot.TopSlideExtendedPositionIN;
                liftTargetIN = 0;
            }

            if (gamepad1.a) {                  //    go to capstone pickup position

                DesiredSuperStructureState = 4;

                liftRequest = true;
                topSlideRequest = true;
                wristRequest = true;
                liftTargetIN = robot.MinimumTopSlideMovementHeightIN;
                topSlideTargetIN = robot.TopSlideCapstonePositionIN;
            }

//          End Gamepad State Control code


//          Manual Gripper Control Code


            if (gamepad1.b) {                 // Toggle Gripper no matter the superstructures state
                if (!PreviousGamePad1B) {

                    if (robot.Gripper.getPosition() != robot.GripperOpen) {
                        robot.Gripper.setPosition(robot.GripperOpen);

                    } else if (robot.Gripper.getPosition() != robot.GripperClosed) {
                        robot.Gripper.setPosition(robot.GripperClosed);
                    }

                    PreviousGamePad1B = true;
                }
            } else
                PreviousGamePad1B = false;

//         End Manual Gripper Control Code


//         Manual Intake Code

            if (!gamepad1.right_bumper && Gampepad1RightBumperPreviousLoopState) {
                Gampepad1RightBumperPreviousLoopState = false;
            }

            if (!gamepad1.left_bumper && Gamepad1LeftBumperPreviousLoopState) {
                Gamepad1LeftBumperPreviousLoopState = false;
            }


            if (gamepad1.right_bumper && IntakePreviousLoopState != 1 && !Gampepad1RightBumperPreviousLoopState) {         // when pressed, if intake not intaking, open gripper and intake
                robot.Gripper.setPosition(robot.GripperOpen);
                IntakePreviousLoopState = 1;
                Gampepad1RightBumperPreviousLoopState = true;
            }

            if (gamepad1.right_bumper && IntakePreviousLoopState == 1 && !Gampepad1RightBumperPreviousLoopState) {        // when pressed, if intake intaking, close gripper and turn off intake
                robot.Gripper.setPosition(robot.GripperClosed);
                IntakePreviousLoopState = 0;
                Gampepad1RightBumperPreviousLoopState = true;
            }

            if (gamepad1.left_bumper && IntakePreviousLoopState != 2 && !Gamepad1LeftBumperPreviousLoopState) {            // when pressed, if intake not outtaking, open gripper and outtake
                robot.Gripper.setPosition(robot.GripperOpen);
                IntakePreviousLoopState = 2;
                Gamepad1LeftBumperPreviousLoopState = true;
            }

            if (gamepad1.left_bumper && IntakePreviousLoopState == 2 && !Gamepad1LeftBumperPreviousLoopState) {            // when pressed, if intake outtaking, turn off intake
                IntakePreviousLoopState = 0;
                Gamepad1LeftBumperPreviousLoopState = true;
            }

//            End Manual Intake Code

            liftObstruction = robot.TopSlidePositionIN < robot.MinimumTopSlideMovementHeightIN && (robot.TopSlidePositionIN > 1 && robot.TopSlidePositionIN < 12);
            wristObstruction = robot.TopSlidePositionIN < 14;

            //completion checks
            if (!liftRequest && !topSlideRequest && !wristRequest)
                DesiredSuperStructureState = 0;

            //wrist code
            if (wristRequest) {
                switch (DesiredSuperStructureState) {
                    case 1:
                        robot.Wrist.setPosition(robot.WristCollectionPosition);
                        wristRequest = false;
                        break;
                    case 2:
                        if (!wristObstruction) {
                            robot.Wrist.setPosition(robot.WristNormalDepositPosition);
                            wristRequest = false;
                        }
                        break;
                    case 3:
                        if (!wristObstruction) {
                            robot.Wrist.setPosition(robot.WristLeftPosition);
                            wristRequest = false;
                        }
                        break;
                    case 4:
                        if (!wristObstruction) {
                            robot.Wrist.setPosition(robot.WristLeftPosition);
                            wristRequest = false;
                        }
                        break;

                }
            }

            if (topSlideRequest && !liftObstruction) {

                if (Math.abs(robot.TopSlidePositionIN - TopSlideTargetIN) < .15) {
                    robot.TopSlide.setPosition(robot.TopSlideOffPower);
                    topSlideRequest = false;

                } else if (TopSlideTargetIN == robot.TopSlideIntakePositionIN) {
                    robot.TopSlide.setPosition(robot.TopSlideRetractPower);
                } else
                    robot.TopSlide.setPosition(Range.clip(
                            Range.scale(
                                    (TopSlideTargetIN - robot.TopSlidePositionIN), -1, 1, robot.TopSlideRetractPower, robot.TopSlideExtendPower),
                            robot.TopSlideRetractPower, robot.TopSlideExtendPower));

                if (TopSlideTargetIN == robot.TopSlideIntakePositionIN && robot.TopSlidePositionIN < 3)
                    if (prevTopSlidePosition == -100 || Math.abs(robot.TopSlidePositionIN - prevTopSlidePosition) > 0.05)
                        stallTimer.reset();
                    else if (stallTimer.seconds() > 0.5) {
                        topSlidePositionOffset = robot.TopSlidePositionIN;
                        topSlideRequest = false;
                        robot.TopSlide.setPosition(robot.TopSlideOffPower);
                    }


                prevTopSlidePosition = robot.TopSlidePositionIN;

            } else {
                prevTopSlidePosition = -100;
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
            }

            if (IntakePreviousLoopState == 0) {
                robot.LeftIntake.setPower(0);
                robot.RightIntake.setPower(0);
            } else if (IntakePreviousLoopState == 1) {
                robot.LeftIntake.setPower(1);
                robot.RightIntake.setPower(1);
            } else if (IntakePreviousLoopState == 2) {
                robot.LeftIntake.setPower(-1);
                robot.RightIntake.setPower(-1);
            }

            telemetry.update();

            LoopCount++;

            LoopFrequency = (1000 * LoopCount) / (System.currentTimeMillis() - StartTime);

            if (isStopRequested()) {
                return;
            }

        }

    }
}
