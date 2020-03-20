package org.firstinspires.ftc.teamcode.worlds;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.State.LiftManager;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.worlds.stuff.*;

@TeleOp
public class LiftTester extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;
    LiftManager lift;

    double triggerSum;

    double leftRPM;
    double rightRPM;
    double leftCurrent;
    double rightCurrent;
    double leftCounts;
    double rightCounts;

    double hubVoltage;

    double gearboxRatio = 13.7;
    double spoolDiameterIN = RobotConstants.LiftSpoolDiameterIN;

    double tickDifference;
    double leftliftSpeedIN;
    double rightliftspeedIN;
    double leftinputPower;
    double rightinputPower;

    double leftoutputPower;
    double rightoutputPower;
    double leftEfficiency;
    double rightEfficiency;
    double leftMotorTorque;
    double rightMotorTorque;

    double leftLiftingForceLB;
    double rightLiftingForceLB;

    int leftstuffIndex;
    int rightstuffIndex;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(">", "Initializing");
        telemetry.update();

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.Elbow2, drive.LeftIntake);

        telemetry.addData(">", "Initialization Complete");
        telemetry.update();

        telemetry.addData("lift.LiftPositionIN", () -> lift.LiftPositionIN);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor Encoder Counts", () -> leftCounts);
        telemetry.addData("Right Motor Encoder Counts", () -> rightCounts);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Encoder Count Difference", () -> tickDifference);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor RPM", () -> leftRPM);
        telemetry.addData("Right Motor RPM", () -> rightRPM);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Lift Speed (IN/s)", () -> leftliftSpeedIN);
        telemetry.addData("Right Lift Speed (IN/s)", () -> rightliftspeedIN);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor Input Power (w)", () -> leftinputPower);
        telemetry.addData("Right Motor Input Power (w)", () -> rightinputPower);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor Output Torque", () -> leftMotorTorque);
        telemetry.addData("Right Motor Output Torque", () -> rightMotorTorque);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor Output Power", () -> leftoutputPower);
        telemetry.addData("Right Motor Output Power", () -> rightoutputPower);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor Efficiency", () -> leftEfficiency);
        telemetry.addData("Right Motor Efficiency", () -> rightEfficiency);
        telemetry.addLine();   //this should create an empty line to improve readability
        telemetry.addData("Left Motor Lifting Force (LB)", () -> leftLiftingForceLB);
        telemetry.addData("Right Motor Lifting Force (LB)", () -> rightLiftingForceLB);

        waitForStart();

        while (opModeIsActive()) {

            triggerSum = (gamepad1.right_trigger * Math.abs(gamepad1.right_trigger)) - ((Math.abs(gamepad1.left_trigger) * gamepad1.left_trigger));

            RevBulkData BulkData2 = drive.hub2.getBulkInputData();

            leftRPM = 60 * BulkData2.getMotorVelocity(drive.LeftLift) / (28 * gearboxRatio);
            rightRPM = 60 * BulkData2.getMotorVelocity(drive.RightLift) / (28 * gearboxRatio);

            leftCounts = BulkData2.getMotorCurrentPosition(drive.LeftLift);
            rightCounts = BulkData2.getMotorCurrentPosition(drive.RightLift);

            tickDifference = Math.abs(leftCounts - rightCounts);

            hubVoltage = drive.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);

            leftCurrent = drive.LeftLift.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            rightCurrent = drive.RightLift.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);

            leftinputPower = hubVoltage * leftCurrent;
            rightinputPower = hubVoltage * rightCurrent;

            leftliftSpeedIN = leftRPM * spoolDiameterIN * Math.PI;
            rightliftspeedIN = rightRPM * spoolDiameterIN * Math.PI;

            leftstuffIndex = getIndex(leftRPM);
            rightstuffIndex = getIndex(rightRPM);

            leftoutputPower = powerOuts[leftstuffIndex];
            rightoutputPower = powerOuts[rightstuffIndex];

            leftEfficiency = efficiencies[leftstuffIndex];
            rightEfficiency = efficiencies[rightstuffIndex];

            leftMotorTorque = torques[leftstuffIndex];
            rightMotorTorque = torques[rightstuffIndex];

            leftLiftingForceLB = (leftMotorTorque * 8.85) / spoolDiameterIN;
            rightLiftingForceLB = (rightMotorTorque * 8.85) / spoolDiameterIN;

            telemetry.update();


        }


    }


}

