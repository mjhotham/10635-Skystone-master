package org.firstinspires.ftc.teamcode.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

@Config
public class LiftManager3 {

    public ExpansionHubMotor LeftLift;
    public ExpansionHubMotor RightLift;
    public ExpansionHubMotor SlideEncoder;
    public ExpansionHubServo Elbow;
    public ExpansionHubServo Elbow2;
    public double liftTargetIN = 0;
    public double slideTargetIN = 0;

    public double externalLiftPositionRequest;
    public double externalSlidePositionRequest;

    public double LiftPositionIN;
    public double SlidePositionIN;

    public double LiftTicksPerInch = RobotConstants.LiftMotorTicksPerRotationofOuputShaft / (RobotConstants.LiftSpoolDiameterIN * Math.PI);
    public double SlideTicksPerInch = RobotConstants.TopslideTicksPerRotation / (RobotConstants.TopslideSpoolDiameterIN * Math.PI);

    double leftLiftPositionIN;
    double rightLiftPositionIN;

    double leftLiftPower;
    double rightLiftPower;

    double SlidePower;

    //tuning this will be a process
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.2, 0, 0);          // starting point for tuning
    public static PIDCoefficients topSlideCoefficients = new PIDCoefficients(0.3, 0, 0);      // starting point for tuning

    public PIDFController leftLiftControl = new PIDFController(liftCoefficients);
    public PIDFController rightLiftControl = new PIDFController(liftCoefficients);
    public PIDFController topSlideControl = new PIDFController(topSlideCoefficients);

    public LiftManager3(ExpansionHubMotor LeftLift, ExpansionHubMotor RightLift, ExpansionHubServo Elbow, ExpansionHubServo Elbow2, ExpansionHubMotor slideEncoder) {
        this.LeftLift = LeftLift;
        this.RightLift = RightLift;
        this.Elbow = Elbow;
        this.Elbow2 = Elbow2;
        this.SlideEncoder = slideEncoder;
        leftLiftControl.setOutputBounds(-1, 1);
        rightLiftControl.setOutputBounds(-1, 1);
        topSlideControl.setOutputBounds(-1, 1);
    }

    public void update(RevBulkData bulkData2) {
        update(bulkData2, 0, false, false);
    }

    public void update(RevBulkData bulkData2, double triggerSum, boolean liftEncoderReset, boolean slideOverride) {

        leftLiftPositionIN = bulkData2.getMotorCurrentPosition(LeftLift) / LiftTicksPerInch;
        rightLiftPositionIN = bulkData2.getMotorCurrentPosition(RightLift) / LiftTicksPerInch;

        LiftPositionIN = Math.max(leftLiftPositionIN, rightLiftPositionIN);
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;

        if (liftEncoderReset) {
            Resetlift();
        }

    }

    void Resetlift() {
        leftLiftPower = 0;
        rightLiftPower = 0;
        LeftLift.setPower(leftLiftPower);
        RightLift.setPower(rightLiftPower);
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}