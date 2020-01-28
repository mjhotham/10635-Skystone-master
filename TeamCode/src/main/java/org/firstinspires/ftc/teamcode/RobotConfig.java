package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotConfig
{
    public DcMotor FrontLeft   = null;
    public DcMotor FrontRight  = null;
    public DcMotor BackLeft    = null;
    public DcMotor BackRight   = null;
    public DcMotor LeftLift = null;
    public DcMotor RightLift = null;
    public DcMotor LeftIntake  = null;
    public DcMotor RightIntake = null;
    public Servo Gripper     = null;
    public Servo Wrist       = null;
    public Servo Elbow       = null;
    public Servo RightHook   = null;
    public Servo LeftHook   = null;
    public Servo LeftAngle = null;
    public Servo RightAngle = null;

    public double GripperOpen = 1;
    public double GripperClosed = .64;

    public double ElbowCollectionPosition = .04;                                  // Dpad_up
    public double ElbowBackLeftDepositPosition = .59;                             // B and X
    public double ElbowFrontRightDepositPosition = .47;                           // A and Y

    public double ElbowExtended = 1600;
    public double ElbowRetract = 0.25;
    public double ElbowExtend = 0.75;

    public double WristCollectionPosition = .11;                                  // Dpad_Up
    public double WristBackDepositPosition = .11;                                 // B
    public double WristFrontDepositPosition = .767;                                  // Y
    public double WristLeftDepositPosition = 1;                                 // X
    public double WristRightDepositPosition = .43;                                  // A

    public double LeftHookDisengaged = .42;                                          //tbd
    public double LeftHookEngaged = .12;                                             // tbd

    public double RightHookDisengaged = .32;                                         //tbd
    public double RightHookEngaged = .62;                                            // tbd


    double SpoolDiameterIN = 1.25;
    double LiftMotorTicksPerRotationofOuputShaft = 537.6;         // for gobilda 19.2:1 Motor

    double LiftTicksPerInch = LiftMotorTicksPerRotationofOuputShaft / (SpoolDiameterIN * Math.PI);

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    public RobotConfig(){


    }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;



        FrontLeft = hwMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hwMap.get(DcMotor.class, "FrontRight");
        BackLeft = hwMap.get(DcMotor.class, "BackLeft");
        BackRight = hwMap.get(DcMotor.class, "BackRight");
        LeftLift = hwMap.get(DcMotor.class,"LeftLift");
        RightLift = hwMap.get(DcMotor.class,"RightLift");
        LeftIntake = hwMap.get(DcMotor.class,"LeftIntake");
        RightIntake = hwMap.get(DcMotor.class,"RightIntake");

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setTargetPosition(LeftLift.getCurrentPosition());
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift.setTargetPosition(RightLift.getCurrentPosition());
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        LeftLift.setPower(0);
        RightLift.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);



        Gripper = hwMap.get(Servo.class, "Gripper");
        Wrist = hwMap.get(Servo.class, "Wrist");
        Elbow = hwMap.get(Servo.class, "Elbow");
        LeftHook = hwMap.get(Servo.class,"LeftHook");
        RightHook = hwMap.get(Servo.class,"RightHook");

        RightAngle = hwMap.get(Servo.class, "RightAngle");
        LeftAngle = hwMap.get(Servo.class, "LeftAngle");

        LeftHook.setPosition(LeftHookDisengaged);
        RightHook.setPosition(RightHookDisengaged);
    }

    int fLeftMotorTarget = 0;
    int fRightMotorTarget = 0;
    int bLeftMotorTarget = 0;
    int bRightMotorTarget = 0;

    final int bettermoveMovementDeadzone = 10;
    final int bettermoveCompletionDeadzone = 50;
    final double turningDistConstant = 32; //adjust this to get the angle right

    double inchesToPulses(double inches) {
        return inches * FrontLeft.getMotorType().getTicksPerRev() / (4 * Math.PI);
    }

    double degreesToPulses(double degrees) {
        return inchesToPulses(degrees * turningDistConstant * Math.PI / 360);
    }

    /***
     * @param forward distance forward in inches
     * @param right distance right straffing in inches *Note: no promises the inches strafing will be accurate or precise due to the physics of strafing
     * @param spin angle to turn clockwise in degrees: will have to be tuned manually by telling it to do a 360 and adjusting turningDistConstant until a 360 is performed
     */
    void setTargets(double forward, double right, double spin) {
        fLeftMotorTarget = FrontLeft.getCurrentPosition() + (int) (inchesToPulses(forward) + inchesToPulses(right * Math.sqrt(2)) + degreesToPulses(spin));
        fRightMotorTarget = FrontRight.getCurrentPosition() + (int) (inchesToPulses(forward) - inchesToPulses(right * Math.sqrt(2)) - degreesToPulses(spin));
        bLeftMotorTarget = BackLeft.getCurrentPosition() + (int) (inchesToPulses(forward) - inchesToPulses(right * Math.sqrt(2)) + degreesToPulses(spin));
        bRightMotorTarget = BackRight.getCurrentPosition() + (int) (inchesToPulses(forward) + inchesToPulses(right * Math.sqrt(2)) - degreesToPulses(spin));
    }

    /***
     * set motor powers to move to position
     * @return true on completion
     */
    boolean bettermove() {
        double fLeftMotorPower = (fLeftMotorTarget - FrontLeft.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - FrontRight.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - BackLeft.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - BackRight.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        double previousMaxMotorPower = Math.max(Math.max(Math.abs(FrontLeft.getPower()), Math.abs(BackLeft.getPower())), Math.max(Math.abs(FrontRight.getPower()), Math.abs(BackRight.getPower())));
        max /= Math.min(Math.min(0.7, previousMaxMotorPower + 0.1), Math.max(Math.abs(max / 1200), 0.1)); //should probably be the best because it doesn't jump to full speed instantly

        if (max > 0) {
            FrontLeft.setPower(fLeftMotorPower / max);
            FrontRight.setPower(fRightMotorPower / max);
            BackLeft.setPower(bLeftMotorPower / max);
            BackRight.setPower(bRightMotorPower / max);
        } else {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }


//        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
        boolean output = Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
//        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, FrontLeft.getPower(), FrontRight.getPower(), BackLeft.getPower(), BackRight.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
        return output;
    }
 }

