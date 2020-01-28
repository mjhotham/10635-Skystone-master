package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


// @TeleOp
public class MattandEvanTeleOpRemastered extends LinearOpMode {


    RobotConfig robot = new RobotConfig();

    double IntakePreviousLoopState = 0;                         //  0 for off   1 for intake   2 for outtake

    boolean Gampepad1RightBumperPreviousLoopState = false;
    boolean Gamepad1LeftBumperPreviousLoopState = false;
    boolean liftActive = false;

    double forward;
    double right;
    double spin;

    double LiftPositionIN;

    double LiftPIDPower = .1;     // Lower this number if the lift acts jerky when you are not touching the controller, raise it if the lift falls when you are not touching the controller

    double MinimumElbowMovementHeightIN = 6;      // Minimum height that the elbow moves in and out of the


    boolean reverseDrivetrain = false;
    boolean grabbed = false;
    boolean slowMode = false;

    boolean previousDpadDown = false;
    boolean previousDpadUp = false;
    boolean Gamepad2RightBumperPreviousLoopState = false;

    boolean previousWheelsOutake = false;

    boolean xToggle = false;
    boolean yToggle = false;

    boolean previousGamepad1y = false;
    boolean previousGamepad1x = false;
    boolean previousGamepad1a = false;
    boolean previousGamepad1b = false;

    double liftPower = 0;

    LynxEmbeddedIMU imu;

    boolean elbowMoving = false;
    boolean elbowExtended = false;

    boolean hooksEngaged = false;

    boolean downRequested = false;

    boolean bMode = false;

    ElapsedTime elbowTimer = new ElapsedTime();
    int elbowEncOffset = 0;
    int prevElbowEnc = 0;

    double snap(double angle) {
        if (Math.abs(angle - (Math.PI / 2)) < Math.PI / 12)
            return Math.PI / 2;
        if (Math.abs(angle + (Math.PI / 2)) < Math.PI / 12)
            return -Math.PI / 2;
        if (Math.abs(angle) + Math.PI / 12 > Math.PI)
            return Math.PI;
        if (Math.abs(angle) < Math.PI / 12)
            return 0;
        return angle;
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

//        imu = hardwareMap.get(LynxEmbeddedIMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.mode = BNO055IMU.SensorMode.MAGGYRO;
//        imu.initialize(parameters);

        robot.Gripper.setPosition(robot.GripperOpen);
        robot.Elbow.setPosition(0.5);
        robot.Wrist.setPosition(robot.WristCollectionPosition);

//        robot.LeftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        idle();
//        robot.LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Say", "Initialization Complete");

        telemetry.addData("Gripper", () -> robot.Gripper.getPosition());
        telemetry.addData("Wrist", () -> robot.Wrist.getPosition());
        telemetry.addData("Elbow", () -> robot.Elbow.getPosition());
        telemetry.addData("Elbow encoder", () -> robot.LeftIntake.getCurrentPosition());//-1700 is extended
        telemetry.addData("ElbowMoving", () -> elbowMoving);
        telemetry.addData("LeftLift Inches", () -> LiftPositionIN);
        telemetry.addData("RightLift Inches", () -> LiftPositionIN);
        telemetry.addData("LeftLift Encoder Counts", () -> robot.LeftLift.getCurrentPosition());
        telemetry.addData("RightLift Encoder Counts", () -> robot.RightLift.getCurrentPosition());
        telemetry.addData("Drive", () -> robot.BackLeft.getCurrentPosition());
//        telemetry.addData("Heading", () -> imu.getAngularOrientation().firstAngle);
        telemetry.update();

//        robot.LeftIntake.setPower(1);000
//        robot.RightIntake.setPower(1);

        waitForStart();

        while (opModeIsActive()) {
            slowMode = gamepad1.left_bumper;

            LiftPositionIN = robot.LeftLift.getCurrentPosition() / robot.LiftTicksPerInch;

            double triggerSum = gamepad1.right_trigger - gamepad1.left_trigger + gamepad2.right_trigger - gamepad2.left_trigger;
            if (Math.abs(triggerSum) > 0.1) {

                liftPower = Range.clip(triggerSum, LiftPositionIN > 0 || gamepad1.back ? Range.clip(liftPower - 0.1, -1, 0) : 0, 1);//limit lift speed
                double liftOffset = (robot.LeftLift.getCurrentPosition() - robot.RightLift.getCurrentPosition()) / (robot.LeftLift.getMotorType().getTicksPerRev());
                robot.LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                robot.RightLift.setPower(liftPower + Math.max(0, liftOffset));

                downRequested = false;

                if (!liftActive) {
                    liftActive = true;
                    robot.LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (LiftPositionIN < MinimumElbowMovementHeightIN) {
                    robot.LeftIntake.setPower(0);
                    robot.RightIntake.setPower(0);
                }
            } else if (downRequested) {

                liftPower = (LiftPositionIN > 0 && !elbowMoving) || LiftPositionIN > MinimumElbowMovementHeightIN * 2 ? Range.clip(liftPower - 0.1, -1, 0) : 0;

                if (elbowMoving && LiftPositionIN < MinimumElbowMovementHeightIN)
                    liftPower = 1;

                double liftOffset = (robot.LeftLift.getCurrentPosition() - robot.RightLift.getCurrentPosition()) / (robot.LeftLift.getMotorType().getTicksPerRev());
                robot.LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                robot.RightLift.setPower(liftPower + Math.max(0, liftOffset));

                if (!liftActive) {
                    liftActive = true;
                    robot.LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if (LiftPositionIN < 0 && !elbowMoving)
                    downRequested = false;

            } else if (gamepad1.back) {
                robot.LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftActive = false;
            } else if (liftActive) {
                liftActive = false;
                int target = Math.max(robot.LeftLift.getCurrentPosition(), robot.RightLift.getCurrentPosition());
                robot.LeftLift.setTargetPosition(target);
                robot.LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LeftLift.setPower(LiftPIDPower);
                robot.RightLift.setTargetPosition(target);
                robot.RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RightLift.setPower(LiftPIDPower);
            }

            forward = gamepad1.left_stick_y * (reverseDrivetrain ? 1 : -1) * (slowMode ? 0.4 : 1);
            right = gamepad1.left_stick_x * (reverseDrivetrain ? -1 : 1) * (slowMode ? 0.4 : 1);
//            if (slowMode)
            spin = gamepad1.right_stick_x * (slowMode ? 0.5 : 1);

            // Get gyro angle in radians
//            double deltatheta = theta - snap(Math.atan2(-gamepad1.right_stick_x, -gamepad1.right_stick_y));
//            deltatheta += (deltatheta > Math.PI) ? -2 * Math.PI : (deltatheta < -Math.PI) ? 2 * Math.PI : 0;
//            if (!slowMode)
//                spin = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) > 0.2 ? deltatheta * Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) : 0;

//            telemetry.addData("Target Theta", Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y));

//            if(Math.abs(spin) < 0.1 && (Math.abs(forward) > 0.1 || Math.abs(right) > 0.1))
//                spin = Range.clip(Math.atan2(forward,right) - theta, -1, 1);

            // rotate coordinate system
//            if (!slowMode) {
//            double theta = imu.getAngularOrientation().firstAngle;
//            double temp = forward * Math.cos(theta) - right * Math.sin(theta);
//            right = (forward * Math.sin(theta) + right * Math.cos(theta)) * Math.sqrt(2);
//            forward = temp;
//            }

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

            if (gamepad1.right_bumper) {
                if (!Gampepad1RightBumperPreviousLoopState) {
                    Gampepad1RightBumperPreviousLoopState = true;

                    if (!grabbed && robot.LeftIntake.getPower() <= 0 && LiftPositionIN < MinimumElbowMovementHeightIN)
                        grabbed = false;
                    else
                        grabbed = !grabbed;

                    robot.Gripper.setPosition(grabbed ? robot.GripperClosed : robot.GripperOpen);

                    if (!grabbed && LiftPositionIN < MinimumElbowMovementHeightIN) {
                        robot.LeftIntake.setPower(1);
                        robot.RightIntake.setPower(1);
                    } else {
                        robot.LeftIntake.setPower(0);
                        robot.RightIntake.setPower(0);
                    }
                }
            } else
                Gampepad1RightBumperPreviousLoopState = false;

            if (gamepad1.dpad_down) {
                if (!previousDpadDown) {
                    bMode = false;
                    if (!grabbed) {
                        previousDpadDown = true;
                        downRequested = true;
                        robot.Wrist.setPosition(robot.WristCollectionPosition);
                        elbowExtended = false;
                        elbowMoving = true;
                        robot.Elbow.setPosition(elbowExtended ? robot.ElbowExtend : robot.ElbowRetract);
                    } else {
                        grabbed = false;
                        robot.Gripper.setPosition(robot.GripperOpen);
                    }
                }
            } else
                previousDpadDown = false;

            if (gamepad1.dpad_up) {
                if (!previousDpadUp) {
                    previousDpadUp = true;
                    hooksEngaged = !hooksEngaged;
                    robot.LeftHook.setPosition(hooksEngaged ? robot.LeftHookEngaged : robot.LeftHookDisengaged);
                    robot.RightHook.setPosition(hooksEngaged ? robot.RightHookEngaged : robot.RightHookDisengaged);
                }
            } else
                previousDpadUp = false;

            if (gamepad1.a) {
                if (!previousGamepad1a) {
                    previousGamepad1a = true;
                    if (bMode) {
//                        if (robot.Wrist.getPosition() == robot.WristBackDepositPosition)
                        robot.Wrist.setPosition(robot.WristFrontDepositPosition);
//                        else
//                            robot.Wrist.setPosition(robot.WristBackDepositPosition);
                    } else {
                        robot.Wrist.setPosition(robot.WristBackDepositPosition);
                        elbowExtended = !elbowExtended;
                        elbowMoving = true;
                        robot.Elbow.setPosition(elbowExtended ? robot.ElbowExtend : robot.ElbowRetract);
//                        elbowTimer.reset();
                    }
                }
            } else
                previousGamepad1a = false;

            if (bMode) {
//                double elbowPosition = robot.LeftIntake.getCurrentPosition();
                if (gamepad1.y) {
                    robot.Elbow.setPosition(robot.ElbowExtend);
                    elbowMoving = false;
                } else if (gamepad1.b)
                    robot.Elbow.setPosition(robot.ElbowRetract);
//                else if (elbowMoving) {
//                    if (elbowPosition < 700)
//                        robot.Elbow.setPosition((2 * robot.ElbowExtend + robot.ElbowRetract) / 3);
//                     else if (elbowPosition < 750) {
//                        robot.Elbow.setPosition(0.5);
//                        if (gamepad1.b)
//                            elbowMoving = false;
//                    }
                else
                    robot.Elbow.setPosition(0.5);
            } else if (elbowMoving) {
                int elbowPosition = robot.LeftIntake.getCurrentPosition() - elbowEncOffset;
                if (prevElbowEnc != elbowPosition || elbowTimer.seconds() > 2)
                    elbowTimer.reset();
                prevElbowEnc = elbowPosition;
                if (elbowTimer.milliseconds() > 100) {
//                    elbowEncOffset = robot.LeftIntake.getCurrentPosition();
//                }
//                if (elbowExtended ? elbowPosition > robot.ElbowExtended : elbowPosition < 10) {
                    robot.Elbow.setPosition(0.5);
                    robot.Elbow.setPosition(0.5);
                    robot.Elbow.setPosition(0.5);
                    elbowMoving = false;
                }
            }

            if (gamepad1.b) {
                if (!previousGamepad1b) {
                    previousGamepad1b = true;
                    robot.Wrist.setPosition(robot.WristRightDepositPosition);
                    if (!bMode && robot.Wrist.getPosition() == robot.WristRightDepositPosition) {
                        elbowMoving = true;
                        bMode = true;
                        robot.Elbow.setPosition(robot.ElbowRetract);
                    }
                }
            } else
                previousGamepad1b = false;

            if (gamepad1.x) {
                robot.LeftIntake.setPower(-1);
                robot.RightIntake.setPower(-1);
                slowMode = true;
            }

            if (gamepad1.y && !bMode)
                robot.Wrist.setPosition(robot.WristFrontDepositPosition);

//        if (gamepad1.right_bumper && IntakePreviousLoopState != 1 && !Gampepad1RightBumperPreviousLoopState) {
//            robot.LeftIntake.setPower(-1);
//            robot.RightIntake.setPower(1);
//            IntakePreviousLoopState = 1;
//            Gampepad1RightBumperPreviousLoopState = true;
//        }
//
//        if (gamepad1.right_bumper && IntakePreviousLoopState == 1 && !Gampepad1RightBumperPreviousLoopState) {
//            robot.LeftIntake.setPower(0);
//            robot.RightIntake.setPower(0);
//            IntakePreviousLoopState = 0;
//            Gampepad1RightBumperPreviousLoopState = true;
//        }
//
//        if (gamepad1.left_bumper && IntakePreviousLoopState != 2 && !Gamepad1LeftBumperPreviousLoopState) {
//            robot.LeftIntake.setPower(1);
//            robot.RightIntake.setPower(-1);
//            IntakePreviousLoopState = 2;
//            Gamepad1LeftBumperPreviousLoopState = true;
//        }
//
//        if (gamepad1.left_bumper && IntakePreviousLoopState == 2 && !Gamepad1LeftBumperPreviousLoopState) {
//            robot.LeftIntake.setPower(0);
//            robot.RightIntake.setPower(0);







//            IntakePreviousLoopState = 0;
//            Gamepad1LeftBumperPreviousLoopState = true;
//        }

//        if (gamepad1.dpad_up) {
//            robot.Gripper.setPosition(GripperOpen);
//            robot.Elbow.setPosition(ElbowCollectionPosition);
//            robot.Wrist.setPosition(WristCollectionPosition);
//        }

/*

        if (gamepad1.b && robot.Elbow.getPosition() != ElbowCollectionPosition) {
//            robot.Elbow.setPosition(ElbowBackLeftDepositPosition);
//            robot.Wrist.setPosition(WristBackDepositPosition);
//        } else if(gamepad1.b && LiftPositionIN > MinimumElbowMovementHeightIN) {
//            robot.Elbow.setPosition(ElbowBackLeftDepositPosition);
//            robot.Wrist.setPosition(WristBackDepositPosition);
//        }
//
//        if (gamepad1.y && robot.Elbow.getPosition() != ElbowCollectionPosition) {
//            robot.Elbow.setPosition(ElbowFrontRightDepositPosition);
//            robot.Wrist.setPosition(WristFrontDepositPosition);
//        } else if(gamepad1.y && LiftPositionIN > MinimumElbowMovementHeightIN){
//            robot.Elbow.setPosition(ElbowFrontRightDepositPosition);
//            robot.Wrist.setPosition(WristFrontDepositPosition);
//        }
//
//        if (gamepad1.x && robot.Elbow.getPosition() != ElbowCollectionPosition) {
//            robot.Elbow.setPosition(ElbowBackLeftDepositPosition);
//            robot.Wrist.setPosition(WristLeftDepositPosition);
//        } else if(gamepad1.x && LiftPositionIN > MinimumElbowMovementHeightIN){
//            robot.Elbow.setPosition(ElbowBackLeftDepositPosition);
//            robot.Wrist.setPosition(WristLeftDepositPosition);
//        }
//
//        if (gamepad1.a && robot.Elbow.getPosition() != ElbowCollectionPosition) {
//            robot.Elbow.setPosition(ElbowFrontRightDepositPosition);
//            robot.Wrist.setPosition(WristRightDepositPosition);
//        } else if(gamepad1.a && LiftPositionIN > MinimumElbowMovementHeightIN) {
//            robot.Elbow.setPosition(ElbowFrontRightDepositPosition);
//            robot.Wrist.setPosition(WristRightDepositPosition);
//        }


 */

//            if (gamepad1.b) {
//                if(LiftPositionIN > MinimumElbowMovementHeightIN)
//                robot.Elbow.setPosition(robot.ElbowBackLeftDepositPosition);
//                robot.Wrist.setPosition(robot.WristBackDepositPosition);
//            }

//            if (gamepad1.y) {
//                if (!previousGamepad1y) {
//                    if (yToggle) {
//                        if (LiftPositionIN > 0) {
//                            robot.Elbow.setPosition(robot.ElbowFrontRightDepositPosition);
//                            robot.Wrist.setPosition(robot.WristFrontDepositPosition);
//                        }
//                    } else {
//                        if (LiftPositionIN > 0) {
//                            robot.Elbow.setPosition(robot.ElbowFrontRightDepositPosition);
//                            robot.Wrist.setPosition(robot.WristRightDepositPosition);
//                        }
//                    }
//                    yToggle = !yToggle;
//                    previousGamepad1y = true;
//                }
//            } else
//                previousGamepad1y = false;
//
//            if (gamepad1.x) {
//                if (!previousGamepad1x) {
//                    if (xToggle) {
//                        if (LiftPositionIN > 0) {
//                            robot.Elbow.setPosition(robot.ElbowBackLeftDepositPosition);
//                            robot.Wrist.setPosition(robot.WristLeftDepositPosition);
//                        }
//                    } else {
//                        if (LiftPositionIN > 0) {
//                            robot.Elbow.setPosition(robot.ElbowBackLeftDepositPosition);
//                            robot.Wrist.setPosition(robot.WristBackDepositPosition);
//                        }
//                    }
//                    xToggle = !xToggle;
//                    previousGamepad1x = true;
//                }
//            } else
//                previousGamepad1x = false;

//            if (gamepad1.a) {
//                if(LiftPositionIN > MinimumElbowMovementHeightIN)
//                robot.Elbow.setPosition(robot.ElbowFrontRightDepositPosition);
//                robot.Wrist.setPosition(robot.WristRightDepositPosition);
//            }

//            if (gamepad2.left_bumper) {
//                if (!previousWheelsOutake) {
//                    robot.LeftIntake.setPower(-1);
//                    robot.RightIntake.setPower(-1);
//                    previousWheelsOutake = true;
//                }
//            } else if (previousWheelsOutake) {
//                if (LiftPositionIN < MinimumElbowMovementHeightIN) {
//                    robot.LeftIntake.setPower(1);
//                    robot.RightIntake.setPower(1);
//                } else {
//                    robot.LeftIntake.setPower(0);
//                    robot.RightIntake.setPower(0);
//                }
//                previousWheelsOutake = false;
//            }
//
//            if (gamepad2.right_bumper) {
//                if (!Gamepad2RightBumperPreviousLoopState) {
//                    robot.LeftIntake.setPower(1);
//                    robot.RightIntake.setPower(1);
//                    Gamepad2RightBumperPreviousLoopState = true;
//                }
//            } else if (Gamepad2RightBumperPreviousLoopState) {
//                Gamepad2RightBumperPreviousLoopState = false;
//                robot.LeftIntake.setPower(0);
//                robot.RightIntake.setPower(0);
//            }


//        if (gamepad1.dpad_left) {
//            robot.Gripper.setPosition(GripperClosed);
//        }
//
//        if (gamepad1.dpad_right) {
//            robot.Gripper.setPosition(GripperOpen);
//        }


            // The following code is used to find servo values


            if (gamepad2.a) {
                robot.Wrist.setPosition(robot.Wrist.getPosition() - .001);
            }

            if (gamepad2.y) {
                robot.Wrist.setPosition(robot.Wrist.getPosition() + .001);
            }

            if (gamepad2.dpad_up)
                robot.Elbow.setPosition(0.2);//retract

            if (gamepad2.dpad_down)
                robot.Elbow.setPosition(0.8);//extend

            if (gamepad2.dpad_left)
                robot.Elbow.setPosition(0.5);//stop

            if (gamepad2.x) {
                robot.Gripper.setPosition(robot.Gripper.getPosition() - .001);
            }

            if (gamepad2.b) {
                robot.Gripper.setPosition(robot.Gripper.getPosition() + .001);
            }

            telemetry.update();
        }
    }


//    @Override
//    public void stop() {
//
//        robot.FrontLeft.setPower(0);
//        robot.FrontRight.setPower(0);
//        robot.BackLeft.setPower(0);
//        robot.BackRight.setPower(0);
//        robot.LeftLift.setPower(0);
//        robot.LeftIntake.setPower(0);
//        robot.RightIntake.setPower(0);
//        https://www.reddit.com/r/FTC/comments/63qyg8/help_measure_encoderdrive_speed/dg1vmyz/


//    }
}
