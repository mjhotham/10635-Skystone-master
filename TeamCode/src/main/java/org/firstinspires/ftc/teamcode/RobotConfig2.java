package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


public class RobotConfig2 {

    public RevBulkData bulkData, bulkData2;
    public ExpansionHubMotor FrontLeft, FrontRight, BackLeft, BackRight, LeftLift, RightLift, RightIntake, LeftIntake;
    public ExpansionHubEx ExpansionHub1, ExpansionHub2;

    public Servo Gripper = null;
    public Servo Wrist = null;
    public Servo TopSlide = null;
    public Servo RightHook = null;
    public Servo LeftHook = null;
    public Servo LeftAngle = null;
    public Servo RightAngle = null;
    public Servo CapStoneLift = null;

    public double GripperOpen = 1;
    public double GripperClosed = .65;

    public double TopSlideExtendedPositionIN = 14;
    public double TopSlideIntakePositionIN = 0;
    public double TopSlideCapstonePositionIN = 10;  // have no clue what this should be

    public double TopSlideRetractPower = 0.2;
    public double TopSlideExtendPower = 0.8;
    public double TopSlideOffPower = 0.5;

    public double WristCollectionPosition = .11;
    public double WristNormalDepositPosition = .78;           // 180 degrees from intake position
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

    double LiftSpoolDiameterIN = 1.25;
    double TopSlideSpoolDiameterIN = 1.25;

    double LiftMotorTicksPerRotationofOuputShaft = 537.6;     // for gobilda 19.7:1 Motor
    double TopSlideTicksPerRoatationOfVexle = 360;            // for Vex Optical shaft encoder

    double LiftTicksPerInch = LiftMotorTicksPerRotationofOuputShaft / (LiftSpoolDiameterIN * Math.PI);
    double TopSlideTicksPerInch = TopSlideTicksPerRoatationOfVexle / (TopSlideSpoolDiameterIN * Math.PI);

    double MinimumTopSlideMovementHeightIN = 6;      // Minimum height that the TopSlide moves in and out of the robot

    double FirstStoneHeight = 2.5;    //  for 2 stone stacking auto

    double SecondStoneHeight = 6.5;  //   for 2 stone stacking auto

    double LeftLiftRPM;
    double RightLiftRPM;

    double TopSlideRPM;

    double LiftPositionIN;

    double TopSlidePositionIN;

    boolean hooksEngaged = false;

    HardwareMap hwMap = null;

    public RobotConfig2() {

    }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        ExpansionHub1 = hwMap.get(ExpansionHubEx.class, "ExpansionHub1");
        ExpansionHub2 = hwMap.get(ExpansionHubEx.class, "ExpansionHub2");

        bulkData = ExpansionHub1.getBulkInputData();
        bulkData2 = ExpansionHub2.getBulkInputData();

        FrontRight = (ExpansionHubMotor) hwMap.dcMotor.get("FrontRight");
        FrontLeft = (ExpansionHubMotor) hwMap.dcMotor.get("FrontLeft");
        BackLeft = (ExpansionHubMotor) hwMap.dcMotor.get("BackLeft");
        BackRight = (ExpansionHubMotor) hwMap.dcMotor.get("BackRight");
        LeftLift = (ExpansionHubMotor) hwMap.dcMotor.get("LeftLift");
        RightLift = (ExpansionHubMotor) hwMap.dcMotor.get("RightLift");
        LeftIntake = (ExpansionHubMotor) hwMap.dcMotor.get("LeftIntake");
        RightIntake = (ExpansionHubMotor) hwMap.dcMotor.get("RightIntake");

        Gripper = hwMap.get(Servo.class, "Gripper");
        Wrist = hwMap.get(Servo.class, "Wrist");
        TopSlide = hwMap.get(Servo.class, "Elbow");
        LeftHook = hwMap.get(Servo.class, "LeftHook");
        RightHook = hwMap.get(Servo.class, "RightHook");
        CapStoneLift = hwMap.get(Servo.class, "CapStoneLift");

        RightAngle = hwMap.get(Servo.class, "RightAngle");
        LeftAngle = hwMap.get(Servo.class, "LeftAngle");

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

        LeftHook.setPosition(LeftHookDisengaged);
        RightHook.setPosition(RightHookDisengaged);
    }



}
