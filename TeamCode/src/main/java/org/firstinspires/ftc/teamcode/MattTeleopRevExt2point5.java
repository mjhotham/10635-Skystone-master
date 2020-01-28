package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.RevBulkData;


// @TeleOp
public class MattTeleopRevExt2point5 extends LinearOpMode {   // revext2 with additional angles code and other interesting changes


    RobotConfig2 robot = new RobotConfig2();

    public RevBulkData bulkData, bulkData2;

    double forward;
    double right;
    double spin;

    double triggerSum;

    double TopSlideTargetIN;

    double liftPower = 0;

    double LoopFrequency;

//    double TopSlideScalar;      // topslide power proportional to topslide distance to target

    int IntakePreviousLoopState = 0;              // 0 for off      1 for intake      2 for outtake

    int DesiredSuperStructureState = 0;           // 0 for manual mode (for lift)    1  for intake position     2 for regular deposit     3 for 90 degree deposit    4 for capstone pickup position

    int LiftEncoderDifference;

    int LiftTarget;

//    int LoopCount = 0;

    long LoopStartTime;

    boolean slowMode = false;

    boolean reverseDrivetrain = false;

    boolean previousDpadUp = false;

    boolean PreviousGamePad1B = false;

    boolean previousLeftStickButton = false;

    boolean previousRightStickButton = false;

    boolean TopSlideMoving = false;

    boolean LiftUnderManualControl = false;

    boolean Gampepad1RightBumperPreviousLoopState = false;

    boolean Gamepad1LeftBumperPreviousLoopState = false;

    ElapsedTime stallTimer = new ElapsedTime();


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

        robot.LeftAngle.setPosition(robot.LeftAngleIntake);
        robot.RightAngle.setPosition(robot.RightAngleIntake);

        telemetry.addData("Say", "Initialization Complete");
//        telemetry.addData("Gripper Position", () -> robot.Gripper.getPosition());
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


        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            LoopStartTime = System.currentTimeMillis();

            if (isStopRequested()) {      // spamming this doesnt fix the crashing on stop, it just slows my loop time
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

            robot.TopSlidePositionIN = bulkData2.getMotorCurrentPosition(robot.LeftIntake) / robot.TopSlideTicksPerInch;

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


            forward = gamepad1.left_stick_y * (reverseDrivetrain ? 1 : -1) * (slowMode ? 0.3 : 1);
            right = gamepad1.left_stick_x * (reverseDrivetrain ? -1 : 1) * (slowMode ? 0.3 : 1);
            spin = gamepad1.right_stick_x * (slowMode ? 0.4 : 1);

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

            if (gamepad2.right_bumper)
                robot.TopSlide.setPosition(Range.scale(gamepad2.left_stick_y,-1,1,.2,.8));

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

                if (Math.abs(robot.Gripper.getPosition() - (robot.GripperClosed)) < .01)
                    robot.LeftAngle.setPosition(robot.LeftAngleOpen);
                    robot.RightAngle.setPosition(robot.RightAngleOpen);

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

                robot.LeftAngle.setPosition(robot.LeftAngleIntake);
                robot.RightAngle.setPosition(robot.RightAngleIntake);

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

            }
//                End Manual Lift Code


            if (gamepad1.dpad_left) {
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                robot.LeftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else
                robot.LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//          Gamepad State control Code

            if (gamepad1.dpad_down) {            //  return to ready to intake

                if (!LiftUnderManualControl) {           // Maybe Unnecessary
                    DesiredSuperStructureState = 1;
                }
            }

            if (gamepad1.y) {                   // deliver with 180 spin

                DesiredSuperStructureState = 2;

            }

            if (gamepad1.x) {                   //    deliver with 90 spin

                DesiredSuperStructureState = 3;

            }



/*
            if (gamepad1.a) {                  //    go to capstone pickup position

                DesiredSuperStructureState = 4;

            }
*/



//          End Gamepad State Control code


//          Manual Gripper Control Code


            if (gamepad1.b) {                 // Toggle Gripper no matter the superstructures state
                if (!PreviousGamePad1B) {

                    if (robot.Gripper.getPosition() != robot.GripperOpen) {
                        robot.Gripper.setPosition(robot.GripperOpen);
                        robot.LeftAngle.setPosition(robot.LeftAngleIntake);
                        robot.RightAngle.setPosition(robot.RightAngleIntake);

                    } else if (robot.Gripper.getPosition() != robot.GripperClosed) {
                        robot.Gripper.setPosition(robot.GripperClosed);
                        robot.LeftAngle.setPosition(robot.LeftAngleGripped);  // not sure if these are necessary
                        robot.RightAngle.setPosition(robot.RightAngleGripped);
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
                robot.LeftAngle.setPosition(robot.LeftAngleIntake);
                robot.RightAngle.setPosition(robot.RightAngleIntake);
                IntakePreviousLoopState = 1;
                Gampepad1RightBumperPreviousLoopState = true;
            }

            if (gamepad1.right_bumper && IntakePreviousLoopState == 1 && !Gampepad1RightBumperPreviousLoopState) {        // when pressed, if intake intaking, close gripper and turn off intake
                robot.Gripper.setPosition(robot.GripperClosed);
                robot.LeftAngle.setPosition(robot.LeftAngleGripped);
                robot.RightAngle.setPosition(robot.RightAngleGripped);
                IntakePreviousLoopState = 0;
                Gampepad1RightBumperPreviousLoopState = true;
            }

            if (gamepad1.left_bumper && IntakePreviousLoopState != 2 && !Gamepad1LeftBumperPreviousLoopState) {            // when pressed, if intake not outtaking, open gripper and outtake
                robot.LeftAngle.setPosition(robot.LeftAngleIntake);
                robot.RightAngle.setPosition(robot.RightAngleIntake);
                robot.Gripper.setPosition(robot.GripperOpen);
                IntakePreviousLoopState = 2;
                Gamepad1LeftBumperPreviousLoopState = true;
            }

            if (gamepad1.left_bumper && IntakePreviousLoopState == 2 && !Gamepad1LeftBumperPreviousLoopState) {            // when pressed, if intake outtaking, turn off intake
                IntakePreviousLoopState = 0;
                Gamepad1LeftBumperPreviousLoopState = true;
            }

//            End Manual Intake Code


            GoToIntakePositionHandler();
            GoToStandardDepositPositionHandler();
            GoTo90DegreeDepositPositionHandler();
//            GoToCapstonePickupHandler();
            IntakeStateHandler();
            TopSlideMovementHandler();


            telemetry.update();

//            LoopCount++;

            LoopFrequency = 1000 / (System.currentTimeMillis() - LoopStartTime);

            if (isStopRequested()) {
                return;
            }

        }

    }


    public void MoveLift(double RequestedHeightIN) {

        if (!LiftUnderManualControl) {

            LiftTarget = (int) Math.round(RequestedHeightIN * robot.LiftTicksPerInch);

            robot.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.LeftLift.setTargetPosition(LiftTarget);
            robot.RightLift.setTargetPosition(LiftTarget);

            liftPower = 1;

            double liftOffset = (bulkData2.getMotorCurrentPosition(robot.LeftLift) - bulkData2.getMotorCurrentPosition(robot.RightLift)) / (robot.LeftLift.getMotorType().getTicksPerRev());

            robot.LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
            robot.RightLift.setPower(liftPower + Math.max(0, liftOffset));

        }

    }


    public void MoveTopSlide(double DesiredTopSlidePositionIN) {


//        if (robot.TopSlidePositionIN >= (DesiredTopSlidePositionIN + .14)) {
//            robot.TopSlide.setPosition(robot.TopSlideRetractPower);
        TopSlideTargetIN = DesiredTopSlidePositionIN;
        TopSlideMoving = true;

//        } else if (robot.TopSlidePositionIN <= (DesiredTopSlidePositionIN - .14)) {
//            robot.TopSlide.setPosition(robot.TopSlideExtendPower);
//            TopSlideTargetIN = DesiredTopSlidePositionIN;
//            TopSlideMoving = true;
//
//        } else {
//            robot.TopSlide.setPosition(robot.TopSlideOffPower);
//            TopSlideMoving = false;
//        }

    }

    double prevTopSlidePosition = -100;
    double topSlidePositionOffset = 0;

    public void TopSlideMovementHandler() {

        if (TopSlideMoving) {

            if (Math.abs((robot.TopSlidePositionIN - topSlidePositionOffset) - TopSlideTargetIN) < .15) {       // turns off the handler when the topslide gets to within a certain value ( .15 in ) of the target
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                TopSlideMoving = false;

            } else if (TopSlideTargetIN == robot.TopSlideIntakePositionIN) {          // this is the only line in the file that would set it to .2
                robot.TopSlide.setPosition(robot.TopSlideRetractPower);                //it is only supposed to happen until the stall code changes TopSlideMoving to false
                //you're code is doing something somewhere
            } else
                robot.TopSlide.setPosition(Range.clip(       // nvm
                        Range.scale(
                                (TopSlideTargetIN - (robot.TopSlidePositionIN - topSlidePositionOffset)), -1, 1, robot.TopSlideRetractPower, robot.TopSlideExtendPower),
                        robot.TopSlideRetractPower, robot.TopSlideExtendPower));

            if (TopSlideTargetIN == robot.TopSlideIntakePositionIN && robot.TopSlidePositionIN - topSlidePositionOffset < 3)
                if (prevTopSlidePosition == -100 || Math.abs(robot.TopSlidePositionIN - prevTopSlidePosition) > 0.05)
                    stallTimer.reset();
                else if (stallTimer.milliseconds() > 100) {
                    topSlidePositionOffset = robot.TopSlidePositionIN;
                    TopSlideMoving = false;
                    robot.TopSlide.setPosition(robot.TopSlideOffPower);
                }


            prevTopSlidePosition = robot.TopSlidePositionIN;

        } else
            prevTopSlidePosition = -100;

    }


    public void GoToIntakePositionHandler() {       // works perfectly #oldcomemnt

        if (DesiredSuperStructureState == 1) {

            robot.LeftAngle.setPosition(robot.LeftAngleIntake);
            robot.RightAngle.setPosition(robot.RightAngleIntake);

            IntakePreviousLoopState = 0;

            if (robot.LiftPositionIN >= robot.MinimumTopSlideMovementHeightIN) {
                MoveTopSlide(robot.TopSlideIntakePositionIN);
                MoveLift(robot.MinimumTopSlideMovementHeightIN);
                robot.Wrist.setPosition(robot.WristCollectionPosition);
            }
            if (robot.LiftPositionIN < robot.MinimumTopSlideMovementHeightIN && robot.TopSlidePositionIN - topSlidePositionOffset > .75) {   // if lift lower than min height and slide not close to intake position
                MoveLift(robot.MinimumTopSlideMovementHeightIN);
            }

            if (robot.LiftPositionIN <= robot.MinimumTopSlideMovementHeightIN && robot.TopSlidePositionIN - topSlidePositionOffset < .75) {   //  handles an unlikely edge case
                MoveTopSlide(robot.TopSlideIntakePositionIN);
                MoveLift(0);
                robot.Wrist.setPosition(robot.WristCollectionPosition);

            } else if (Math.abs(robot.TopSlidePositionIN - (robot.TopSlideIntakePositionIN - topSlidePositionOffset)) < .5) {
                MoveLift(0);
                robot.Gripper.setPosition(robot.GripperOpen);
                robot.Wrist.setPosition(robot.WristCollectionPosition);
            }

            if (!TopSlideMoving && Math.abs(robot.LiftPositionIN) < 1) {           // designed to turn off the handler when done, may be too restrictive
                DesiredSuperStructureState = 0;
                robot.Gripper.setPosition(robot.GripperOpen);
            }

            if (LiftUnderManualControl) {                                                          // if the triggers are pressed the operation is aborted and the TopSlide stops
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

    public void GoToStandardDepositPositionHandler() {     // works perfectly

        if (DesiredSuperStructureState == 2) {

            robot.LeftAngle.setPosition(robot.LeftAngleOpen);
            robot.RightAngle.setPosition(robot.RightAngleOpen);

            robot.Gripper.setPosition(robot.GripperClosed);
            IntakePreviousLoopState = 0;

            if (robot.LiftPositionIN < robot.MinimumTopSlideMovementHeightIN - .15) {
                MoveLift(robot.MinimumTopSlideMovementHeightIN);
            }

            if (robot.LiftPositionIN >= robot.MinimumTopSlideMovementHeightIN - .25) {
                MoveTopSlide(robot.TopSlideExtendedPositionIN);
            }

            if (robot.TopSlidePositionIN > 12) {
                robot.Wrist.setPosition(robot.WristNormalDepositPosition);
            }

            if (Math.abs(robot.TopSlidePositionIN - topSlidePositionOffset - robot.TopSlideExtendedPositionIN) < .15) {      // designed to turn off the handler (Mainly to allow gripper to be opened), may be too restrictive
                robot.Wrist.setPosition(robot.WristNormalDepositPosition);
                DesiredSuperStructureState = 0;
            }

            if (triggerSum < -.1) {                                      // if the Left trigger is pressed (manual lower lift) the operation is aborted and the TopSlide stops
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

    public void GoTo90DegreeDepositPositionHandler() {      // works perfectly

        if (DesiredSuperStructureState == 3) {

            robot.LeftAngle.setPosition(robot.LeftAngleOpen);
            robot.RightAngle.setPosition(robot.RightAngleOpen);

            robot.Gripper.setPosition(robot.GripperClosed);
            IntakePreviousLoopState = 0;

            if (robot.LiftPositionIN < robot.MinimumTopSlideMovementHeightIN - .15) {
                MoveLift(robot.MinimumTopSlideMovementHeightIN);
            }

            if (robot.LiftPositionIN >= robot.MinimumTopSlideMovementHeightIN - .25) {
                MoveTopSlide(robot.TopSlideExtendedPositionIN);
            }

/*
            if (robot.TopSlidePositionIN > 12) {
                robot.Wrist.setPosition(robot.WristLeftPosition);
            }
*/

            if (Math.abs(robot.TopSlidePositionIN - topSlidePositionOffset - robot.TopSlideExtendedPositionIN) < .15) {          // designed to turn off the handler when its finished (Mainly to allow gripper to be opened), may be too restrictive
                robot.Wrist.setPosition(robot.WristLeftPosition);

                DesiredSuperStructureState = 0;
            }

            if (triggerSum < -.1) {                                 // if the Left trigger is pressed (manual lower lift) the opertation is aborted and the TopSlide stops
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

/*
   public void GoToCapstonePickupHandler() {

        if (DesiredSuperStructureState == 4) {

            robot.LeftAngle.setPosition(robot.LeftAngleOpen);
            robot.RightAngle.setPosition(robot.RightAngleOpen);

            robot.Gripper.setPosition(robot.GripperClosed);
            IntakePreviousLoopState = 0;

            if (robot.LiftPositionIN < robot.MinimumTopSlideMovementHeightIN) {
                MoveLift(robot.MinimumTopSlideMovementHeightIN);

            }
            if (robot.LiftPositionIN >= robot.MinimumTopSlideMovementHeightIN) {             // unlikely to ever be needed but could be used if mason decides he wants to grab the capstone after going to the y or x positions
                MoveTopSlide(robot.TopSlideIntakePositionIN);                                 // moves the stone out of the way of the capstone to prevent it from crossing paths in rare edge cases
                robot.Wrist.setPosition(robot.WristLeftPosition);

            } else if (robot.LiftPositionIN >= robot.MinimumTopSlideMovementHeightIN && Math.abs(robot.TopSlidePositionIN - topSlidePositionOffset - robot.TopSlideIntakePositionIN) < .15) {
                MoveLift(robot.MinimumTopSlideMovementHeightIN);

            } else if (Math.abs(robot.LiftPositionIN - robot.MinimumTopSlideMovementHeightIN) < .15) {
                MoveTopSlide(robot.TopSlideCapstonePositionIN);
                robot.Wrist.setPosition(robot.WristLeftPosition);
            }

            if (Math.abs(robot.TopSlidePositionIN - topSlidePositionOffset - robot.TopSlideCapstonePositionIN) < .15 && Math.abs(robot.LiftPositionIN - robot.MinimumTopSlideMovementHeightIN) < .15) {
            }

            if (LiftUnderManualControl) {                                // if the triggers are pressed the operation is aborted and the TopSlide stops
                robot.TopSlide.setPosition(robot.TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }
*/

    public void IntakeStateHandler() {

        if (IntakePreviousLoopState == 0) {
            robot.LeftIntake.setPower(0);
            robot.RightIntake.setPower(0);
        } else if (IntakePreviousLoopState == 1) {
            robot.LeftIntake.setPower(1);
            robot.RightIntake.setPower(1);
        } else if (IntakePreviousLoopState == 2) {    // may need to reduce this to avoid penalties for launching
            robot.LeftIntake.setPower(-1);
            robot.RightIntake.setPower(-1);
        }
    }
}
