package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

// @TeleOp
public class MattTeleOpRevExt extends LinearOpMode {

    public RevBulkData bulkData, bulkData2;
    public ExpansionHubMotor FrontLeft, FrontRight, BackLeft, BackRight, LeftLift, RightLift, RightIntake, LeftIntake;
    public ExpansionHubEx ExpansionHub1, ExpansionHub2;

    public Servo Gripper = null;
    public Servo Wrist = null;
    public Servo TopSlide = null;
    public Servo RightHook = null;
    public Servo LeftHook = null;

    public double GripperOpen = 1;
    public double GripperClosed = .64;

    public double TopSlideExtendedPositionIN = 16;  // have no clue what this should be
    public double TopSlideIntakePositionIN = 0;     // should be 0 or close to 0
    public double TopSlideCapstonePositionIN = 10;  // also have no clue what this should be

    public double TopSlideRetractPower = 0.2;
    public double TopSlideExtendPower = 0.8;
    public double TopSlideOffPower = 0.5;

    public double WristCollectionPosition = .11;
    public double WristNormalDepositPosition = .767;           // 180 degrees from intake position
    public double WristLeftPosition = .43;                     // 90 degrees from intake position      used for picking up capstone and building towers with teams that build them rotated 90 degrees

    public double LeftHookDisengaged = .42;
    public double LeftHookEngaged = .12;

    public double RightHookDisengaged = .32;
    public double RightHookEngaged = .62;

    public double LeftAngleOpen = 0.69833;
    public double LeftAngleIntake = 0.71722;
    public double LeftAngleGripped = 0.72722;
    public double LeftAngleScanningBlue = .71722;
    public double LeftAngleScanningRed = .68;       // not sure what this should be

    public double RightAngleOpen = 0.80777;
    public double RightAngleIntake = 0.79333;
    public double RightAngleGripped = 0.76944;

    double LiftPIDPower = .1;     // Lower this number if the lift acts jerky when you are not touching the controller, raise it if the lift falls when you are not touching the controller
    double liftPower = 0;

    double LiftSpoolDiameterIN = 1.25;
    double TopSlideSpoolDiameterIN = 1.25;

    double LiftMotorTicksPerRotationofOuputShaft = 753.2;     // for gobilda 26.9:1 Motor
    double TopSlideTicksPerRoatationOfVexle = 360;            // for Vex Optical shaft encoder

    double LiftTicksPerInch = LiftMotorTicksPerRotationofOuputShaft / (LiftSpoolDiameterIN * Math.PI);
    double TopSlideTicksPerInch = TopSlideTicksPerRoatationOfVexle / (TopSlideSpoolDiameterIN * Math.PI);

    double MinimumTopSlideMovementHeightIN = 6;      // Minimum height that the TopSlide moves in and out of the robot

    double LeftLiftRPM;
    double RightLiftRPM;

    double TopSlideRPM;

    double forward;
    double right;
    double spin;

    double LiftPositionIN;

    double TopSlidePositionIN;

    double triggerSum;

    double TopSlideTargetIN;

    double LoopFrequency;

    int IntakePreviousLoopState = 0;              // 0 for off      1 for intake      2 for outtake

    int DesiredSuperStructureState = 0;           // 0 for manual mode (for lift)    1  for intake position     2 for regular deposit     3 for 90 degree deposit    4 for capstone pickup position

    int LiftEncoderDifference;

    int LiftTarget;

    int LoopCount = 0;

    long StartTime;

    long LiftAccelerationStartTime;

    boolean hooksEngaged = false;

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

    boolean LiftJustStartedGoingDownManually = false;


    public void init(MattTeleOpRevExt MattTeleOpRevExt) {

        ExpansionHub1 = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub1");
        ExpansionHub2 = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub2");

        bulkData = ExpansionHub1.getBulkInputData();
        bulkData2 = ExpansionHub2.getBulkInputData();

        FrontRight = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = (ExpansionHubMotor) hardwareMap.dcMotor.get("FrontLeft");
        BackLeft = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackLeft");
        BackRight = (ExpansionHubMotor) hardwareMap.dcMotor.get("BackRight");
        LeftLift = (ExpansionHubMotor) hardwareMap.dcMotor.get("LeftLift");
        RightLift = (ExpansionHubMotor) hardwareMap.dcMotor.get("RightLift");
        LeftIntake = (ExpansionHubMotor) hardwareMap.dcMotor.get("LeftIntake");
        RightIntake = (ExpansionHubMotor) hardwareMap.dcMotor.get("RightIntake");

        Gripper = hardwareMap.get(Servo.class, "Gripper");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        TopSlide = hardwareMap.get(Servo.class, "Elbow");
        LeftHook = hardwareMap.get(Servo.class, "LeftHook");
        RightHook = hardwareMap.get(Servo.class, "RightHook");

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        LeftLift.setDirection(DcMotor.Direction.REVERSE);
        LeftIntake.setDirection(DcMotor.Direction.REVERSE);
        RightLift.setDirection(DcMotor.Direction.FORWARD);
        RightIntake.setDirection(DcMotor.Direction.FORWARD);

        RightLift.setTargetPosition(bulkData2.getMotorCurrentPosition(RightLift));
        LeftLift.setTargetPosition(bulkData2.getMotorCurrentPosition(LeftLift));

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        LeftLift.setPower(0);
        RightLift.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        bulkData = ExpansionHub1.getBulkInputData();
        bulkData2 = ExpansionHub2.getBulkInputData();

        Gripper.setPosition(GripperOpen);
        TopSlide.setPosition(.5);
        Wrist.setPosition(WristCollectionPosition);
        LeftHook.setPosition(LeftHookDisengaged);
        RightHook.setPosition(RightHookDisengaged);

        telemetry.addData("Say", "Initialization Complete");
        telemetry.addData("Gripper Position", () -> Gripper.getPosition());
        telemetry.addData("Wrist Position", () -> Wrist.getPosition());
        telemetry.addData("TopSlide Power", () -> TopSlide.getPosition());
        telemetry.addData("TopSlide encoder", () -> bulkData2.getMotorCurrentPosition(LeftIntake));
        telemetry.addData("TopSlide RPM", () -> TopSlideRPM);                                              // want to know what kind of load the 393 is under
        telemetry.addData("Lift Inches", () -> LiftPositionIN);
        telemetry.addData("LeftLift Power", () -> LeftLift.getPower());
        telemetry.addData("RightLift Power", () -> RightLift.getPower());
        telemetry.addData("LeftLift Encoder Counts", () -> bulkData2.getMotorCurrentPosition(LeftLift));
        telemetry.addData("RightLift Encoder Counts", () -> bulkData2.getMotorCurrentPosition(RightLift));
        telemetry.addData("Lift Encoder Count Difference", () -> LiftEncoderDifference);
        telemetry.addData("left Lift Motor RPM", () -> LeftLiftRPM);
        telemetry.addData("Right Lift Motor RPM", () -> RightLiftRPM);
        telemetry.addData("Loop Frequency", () -> LoopFrequency);


        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            if (LoopCount == 0) {
                StartTime = System.currentTimeMillis();
            }

            if (isStopRequested())
                return;

            bulkData = ExpansionHub1.getBulkInputData();      // not sure which motors are on which hub

            if (isStopRequested())
                return;

            bulkData2 = ExpansionHub2.getBulkInputData();

            if(isStopRequested())
                return;

            LiftPositionIN = (bulkData2.getMotorCurrentPosition(LeftLift) + bulkData2.getMotorCurrentPosition(RightLift)) / (2 * LiftTicksPerInch);

            TopSlidePositionIN = bulkData2.getMotorCurrentPosition(LeftIntake) / TopSlideTicksPerInch;

            LiftEncoderDifference = Math.abs(bulkData2.getMotorCurrentPosition(RightLift) - bulkData2.getMotorCurrentPosition(LeftLift));

            LeftLiftRPM = (bulkData2.getMotorVelocity(LeftLift) * 60) / LiftMotorTicksPerRotationofOuputShaft;            //  assumes getMotorVelocity reports in ticks per second
            RightLiftRPM = (bulkData2.getMotorVelocity(RightLift) * 60) / LiftMotorTicksPerRotationofOuputShaft;          // "                                                     "

            TopSlideRPM = ((bulkData2.getMotorVelocity(LeftIntake)) * 60) / TopSlideTicksPerRoatationOfVexle;             // "                                                     "


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

            FrontLeft.setPower(forward + spin + right);
            FrontRight.setPower(forward - spin - right);
            BackLeft.setPower(forward + spin - right);
            BackRight.setPower(forward - spin + right);

//          End DriveTrain Code


//          Hooks Code

            if (gamepad1.dpad_up) {
                if (!previousDpadUp) {
                    previousDpadUp = true;
                    hooksEngaged = !hooksEngaged;
                    LeftHook.setPosition(hooksEngaged ? LeftHookEngaged : LeftHookDisengaged);
                    RightHook.setPosition(hooksEngaged ? RightHookEngaged : RightHookDisengaged);
                }
            } else
                previousDpadUp = false;

//          End Hooks Code


//           Manual Lift Code

            triggerSum = gamepad1.right_trigger - gamepad1.left_trigger;

            if (Math.abs(triggerSum) > 0.1) {

                if (!LiftUnderManualControl) {
                    LiftUnderManualControl = true;
                    LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }


                liftPower = Range.clip(triggerSum, LiftPositionIN > 0 || gamepad1.back ? Range.clip(liftPower - 0.1, -1, 0) : 0, 1);                  //limit lift speed


                double liftOffset = (bulkData.getMotorCurrentPosition(LeftLift) - bulkData.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

                LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                RightLift.setPower(liftPower + Math.max(0, liftOffset));


            } else if (LiftUnderManualControl) {
                int target = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(RightLift));
                LeftLift.setTargetPosition(target);
                LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLift.setPower(LiftPIDPower);
                RightLift.setTargetPosition(target);
                RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLift.setPower(LiftPIDPower);
                LiftUnderManualControl = false;


            } else if (gamepad1.back) {
                LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

//                End Manual Lift Code


//          Gamepad State control COde

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

            if (gamepad1.a) {                  //    go to capstone pickup position

                DesiredSuperStructureState = 4;

            }

//          End Gamepad State Control code


//          Manual Gripper Control Code


            if (gamepad1.b) {                   // Toggle Gripper no matter the superstructures state
                if (!PreviousGamePad1B) {

                    if (Gripper.getPosition() == GripperClosed) {
                        Gripper.setPosition(GripperOpen);

                    } else if (Gripper.getPosition() == GripperOpen) {
                        Gripper.setPosition(GripperClosed);
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
                Gripper.setPosition(GripperOpen);
                IntakePreviousLoopState = 1;
                Gampepad1RightBumperPreviousLoopState = true;
            }

            if (gamepad1.right_bumper && IntakePreviousLoopState == 1 && !Gampepad1RightBumperPreviousLoopState) {        // when pressed, if intake intaking, close gripper and turn off intake
                Gripper.setPosition(GripperClosed);
                IntakePreviousLoopState = 0;
                Gampepad1RightBumperPreviousLoopState = true;
            }

            if (gamepad1.left_bumper && IntakePreviousLoopState != 2 && !Gamepad1LeftBumperPreviousLoopState) {            // when pressed, if intake not outtaking, open gripper and outtake
                Gripper.setPosition(GripperOpen);
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
            GoToCapstonePickupHandler();
            IntakeStateHandler();
            TopSlideMovementHandler();


            telemetry.update();

            LoopCount++;

            LoopFrequency = (1000 * LoopCount) / (System.currentTimeMillis() - StartTime);

        }

    }


    public void MoveLift(double RequestedHeightIN) {

        if (!LiftUnderManualControl) {

            LiftTarget = (int) Math.round(RequestedHeightIN * LiftTicksPerInch);

            RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftLift.setTargetPosition(LiftTarget);
            RightLift.setTargetPosition(LiftTarget);

            liftPower = 1;

            double liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

            LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
            RightLift.setPower(liftPower + Math.max(0, liftOffset));

        }

    }


    public void MoveTopSlide(double DesiredTopSlidePositionIN) {


        if (TopSlidePositionIN > DesiredTopSlidePositionIN) {
            TopSlide.setPosition(TopSlideRetractPower);
            TopSlideTargetIN = DesiredTopSlidePositionIN;
            TopSlideMoving = true;

        } else if (TopSlidePositionIN < DesiredTopSlidePositionIN) {
            TopSlide.setPosition(TopSlideExtendPower);
            TopSlideTargetIN = DesiredTopSlidePositionIN;
            TopSlideMoving = true;

        } else {
            TopSlide.setPosition(TopSlideOffPower);
            TopSlideMoving = false;
        }

    }


    public void TopSlideMovementHandler() {

        if (TopSlideMoving) {

            if (Math.abs(TopSlidePositionIN - TopSlideTargetIN) < .15) {       // turns off the handler when the topslide gets to within a certain value ( .15 in ) of the target
                TopSlide.setPosition(TopSlideOffPower);
                TopSlideMoving = false;

            } else if (TopSlidePositionIN < TopSlideTargetIN) {
                TopSlide.setPosition(TopSlideExtendPower);
                TopSlideMoving = true;

            } else if (TopSlidePositionIN > TopSlideTargetIN) {
                TopSlide.setPosition(TopSlideRetractPower);
                TopSlideMoving = true;
            }

        }

    }


    public void GoToIntakePositionHandler() {

        if (DesiredSuperStructureState == 1) {

            IntakePreviousLoopState = 0;

            if (LiftPositionIN >= MinimumTopSlideMovementHeightIN) {
                MoveTopSlide(TopSlideIntakePositionIN);
                MoveLift(MinimumTopSlideMovementHeightIN);
                Wrist.setPosition(WristCollectionPosition);

            } else if (LiftPositionIN < MinimumTopSlideMovementHeightIN && TopSlidePositionIN > .75) {   // if lift lower than min height and slide not close to intake position
                MoveLift(MinimumTopSlideMovementHeightIN);

            } else if (LiftPositionIN < MinimumTopSlideMovementHeightIN && TopSlidePositionIN < .75) {   //  handles an unlikely edge case
                MoveTopSlide(TopSlideIntakePositionIN);
                MoveLift(0);
                Wrist.setPosition(WristCollectionPosition);

            } else if (TopSlidePositionIN == TopSlideIntakePositionIN) {
                MoveLift(0);
                Wrist.setPosition(WristCollectionPosition);

            } else if (TopSlidePositionIN == TopSlideIntakePositionIN && LiftPositionIN == 0) {           // designed to turn off the handler when done, may be too restrictive
                DesiredSuperStructureState = 0;
                Gripper.setPosition(GripperOpen);
            }

            if (LiftUnderManualControl) {                                                          // if the triggers are pressed the operation is aborted and the TopSlide stops
                TopSlide.setPosition(TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

    public void GoToStandardDepositPositionHandler() {

        if (DesiredSuperStructureState == 2) {
            Gripper.setPosition(GripperClosed);
            IntakePreviousLoopState = 0;

            if (LiftPositionIN < MinimumTopSlideMovementHeightIN) {
                MoveLift(MinimumTopSlideMovementHeightIN);

            } else if (LiftPositionIN >= MinimumTopSlideMovementHeightIN) {
                MoveTopSlide(TopSlideExtendedPositionIN);
                Wrist.setPosition(WristNormalDepositPosition);

            } else if (TopSlidePositionIN == TopSlideIntakePositionIN && LiftPositionIN == 0) {      // designed to turn off the handler (Mainly to allow gripper to be opened), may be too restrictive
                DesiredSuperStructureState = 0;
            }

            if (triggerSum < -.1) {                                      // if the Left trigger is pressed (manual lower lift) the operation is aborted and the TopSlide stops
                TopSlide.setPosition(TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

    public void GoTo90DegreeDepositPositionHandler() {

        if (DesiredSuperStructureState == 3) {
            Gripper.setPosition(GripperClosed);
            IntakePreviousLoopState = 0;

            if (LiftPositionIN < MinimumTopSlideMovementHeightIN) {
                MoveLift(MinimumTopSlideMovementHeightIN);

            } else if (LiftPositionIN >= MinimumTopSlideMovementHeightIN) {
                MoveTopSlide(TopSlideExtendedPositionIN);
                Wrist.setPosition(WristLeftPosition);

            } else if (TopSlidePositionIN == TopSlideExtendedPositionIN) {          // designed to turn off the handler when its finished (Mainly to allow gripper to be opened), may be too restrictive
                DesiredSuperStructureState = 0;
            }

            if (triggerSum < -.1) {                                 // if the Left trigger is pressed (manual lower lift) the opertation is aborted and the TopSlide stops
                TopSlide.setPosition(TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

    public void GoToCapstonePickupHandler() {                                      // superstructure most likely to be in intake position when this is called

        if (DesiredSuperStructureState == 4) {
            Gripper.setPosition(GripperClosed);
            IntakePreviousLoopState = 0;

            if (LiftPositionIN < MinimumTopSlideMovementHeightIN) {
                MoveLift(MinimumTopSlideMovementHeightIN);

            } else if (LiftPositionIN >= MinimumTopSlideMovementHeightIN) {             // unlikely to ever be needed but could be used if mason decides he wants to grab the capstone after going to the y or x positions
                MoveTopSlide(TopSlideIntakePositionIN);                                 // moves the stone out of the way of the capstone to prevent it from crossing paths in rare edge cases
                Wrist.setPosition(WristLeftPosition);

            } else if (LiftPositionIN >= MinimumTopSlideMovementHeightIN && TopSlidePositionIN == TopSlideIntakePositionIN) {
                MoveLift(MinimumTopSlideMovementHeightIN);

            } else if (LiftPositionIN == MinimumTopSlideMovementHeightIN) {
                MoveTopSlide(TopSlideCapstonePositionIN);
                Wrist.setPosition(WristLeftPosition);

            } else if (TopSlidePositionIN == TopSlideCapstonePositionIN && LiftPositionIN == MinimumTopSlideMovementHeightIN) {        // designed to turn off the handler when its finished (Mainly to allow gripper to be opened), may be too restrictive
                DesiredSuperStructureState = 0;
            }

            if (LiftUnderManualControl) {                                // if the triggers are pressed the operation is aborted and the TopSlide stops
                TopSlide.setPosition(TopSlideOffPower);
                DesiredSuperStructureState = 0;
            }
        }
    }

    public void IntakeStateHandler() {

        if (IntakePreviousLoopState == 0) {
            LeftIntake.setPower(0);
            RightIntake.setPower(0);
        } else if (IntakePreviousLoopState == 1) {
            LeftIntake.setPower(1);
            RightIntake.setPower(1);
        } else if (IntakePreviousLoopState == 2) {
            LeftIntake.setPower(-1);
            RightIntake.setPower(-1);
        }
    }
}

/////YOU FOOL!!! YOU'RE NOT EVEN RUNNING INIT