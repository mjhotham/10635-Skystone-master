package org.firstinspires.ftc.teamcode.State;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

public class LiftManager {

    public ExpansionHubMotor LeftLift;
    public ExpansionHubMotor RightLift;
    public ExpansionHubMotor SlideEncoder;
    public ExpansionHubServo Elbow;
    public ExpansionHubServo Elbow2;
    public double liftTargetIN = 0;
    public double slideTargetIN = 0;
    public boolean isBusy = false;
    public double liftPower = 1;
    public double pidPower = 0.1;

    public double LiftTicksPerInch = RobotConstants.LiftMotorTicksPerRotationofOuputShaft / (RobotConstants.LiftSpoolDiameterIN * Math.PI);
    public double SlideTicksPerInch = RobotConstants.TopslideTicksPerRotation / (RobotConstants.TopslideSpoolDiameterIN * Math.PI);

    public double tolerance = 0.25;
    public double LiftPositionIN = 0;
    public double SlidePositionIN = 0;

    boolean liftObstruction = false;
    boolean slideObstruction = false;

    double liftOffset;

    double leftLiftPositionIN;
    double rightLiftPositionIN;

    double leftLiftPower;
    double rightLiftPower;


    public LiftManager(ExpansionHubMotor LeftLift, ExpansionHubMotor RightLift, ExpansionHubServo Elbow, ExpansionHubServo Elbow2, ExpansionHubMotor slideEncoder) {
        this.LeftLift = LeftLift;
        this.RightLift = RightLift;
        this.Elbow = Elbow;
        this.Elbow2 = Elbow2;
        this.SlideEncoder = slideEncoder;
    }

    public void pause(RevBulkData bulkData2) {
        int target = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(LeftLift));
        LeftLift.setTargetPosition(target);
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift.setPower(pidPower);
        RightLift.setTargetPosition(target);
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift.setPower(pidPower);
        liftPower = pidPower;
        Elbow.setPosition(0.5);
        Elbow2.setPosition(0.5);
    }

    public void stop() {
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void reset() {
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void start(RevBulkData bulkData2) {
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftPositionIN = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(RightLift)) / LiftTicksPerInch;
        liftTargetIN = LiftPositionIN;
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void start(double liftTargetIN) {
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftTargetIN = liftTargetIN;
        isBusy = true;
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void update(RevBulkData bulkData2) {


        LiftPositionIN = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(RightLift)) / LiftTicksPerInch;
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;

        slideObstruction = LiftPositionIN < 8;
        liftObstruction = Math.abs(SlidePositionIN - slideTargetIN) > (LiftPositionIN < 5 ? 4 : 1);

        if (liftObstruction && liftTargetIN <= 8 && LiftPositionIN < 8.3) {
            liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (RobotConstants.LiftMotorTicksPerRotationofOuputShaft);

            liftPower = 8.4 - LiftPositionIN;
            LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
            RightLift.setPower(liftPower + Math.max(0, liftOffset));

        } else if (liftObstruction && liftTargetIN <= 8 && LiftPositionIN < 11) {
            LeftLift.setPower(-0.01);
            RightLift.setPower(-0.01);
        } else {

            //LeftLift.setTargetPosition(LiftTarget);
            //RightLift.setTargetPosition(LiftTarget);


            liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (RobotConstants.LiftMotorTicksPerRotationofOuputShaft);

//            liftPower = (liftTargetIN - LiftPositionIN);
            liftPower = 0.5 * ((liftTargetIN == 0 ? -0.5 : liftTargetIN) - LiftPositionIN);

            LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
            RightLift.setPower(liftPower + Math.max(0, liftOffset));
        }

        isBusy = Math.abs(LiftPositionIN - liftTargetIN) > 1 || liftObstruction;

        if (Math.abs(SlidePositionIN - slideTargetIN) < 0.5) {
            Elbow.setPosition(0.5);
            Elbow2.setPosition(0.5);
        } else {
            if (slideObstruction) {
                Elbow.setPosition(0.5);
                Elbow2.setPosition(0.5);
            } else
                Elbow.setPosition(Range.clip(Range.scale((slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8), 0.2, 0.8));
            Elbow2.setPosition(Range.clip(Range.scale((slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8), 0.2, 0.8));
        }

    }

    ElapsedTime smoothnessTimer = new ElapsedTime();

    public void update(RevBulkData bulkData2, double triggerSum, boolean override, boolean slideOverride) {

        update(bulkData2, triggerSum, override, slideOverride, 0);
    }

    public void update(RevBulkData bulkData2, double triggerSum, boolean override, boolean slideOverride, float tilt) {

        leftLiftPositionIN = bulkData2.getMotorCurrentPosition(LeftLift) / LiftTicksPerInch;
        rightLiftPositionIN = bulkData2.getMotorCurrentPosition(RightLift) / LiftTicksPerInch;

        LiftPositionIN = Math.max(leftLiftPositionIN, rightLiftPositionIN);
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;

        slideObstruction = LiftPositionIN < 6.5;
        liftObstruction = Math.abs(SlidePositionIN - slideTargetIN) > (slideObstruction ? 3 : 1);
//        int LiftTarget = (int) Math.round(liftTargetIN * LiftTicksPerInch);

        if (override) {
            Elbow.setPosition(0.5);
            Elbow2.setPosition(0.5);
            if (Math.abs(triggerSum) > 0.1) {
                LeftLift.setPower(triggerSum);
                RightLift.setPower(triggerSum);
                liftTargetIN = LiftPositionIN;
            } else {
                RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                start(bulkData2);
            }

        } else {

            if (liftObstruction && liftTargetIN <= 7 && LiftPositionIN < 7) {
                liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (RobotConstants.LiftMotorTicksPerRotationofOuputShaft);

                liftPower = 7.4 - LiftPositionIN;
                LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                RightLift.setPower(liftPower + Math.max(0, liftOffset));

            } else if (Math.abs(triggerSum) > 0.1 && (LiftPositionIN > 0 || triggerSum > 0)) {
                liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (RobotConstants.LiftMotorTicksPerRotationofOuputShaft);

                liftPower = triggerSum;
                liftTargetIN = Math.max(LiftPositionIN, 0);

                smoothnessTimer.reset();

                LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                RightLift.setPower(liftPower + Math.max(0, liftOffset));
            } else if (liftObstruction && liftTargetIN <= 7 && LiftPositionIN < 9) {
                LeftLift.setPower(0);
                RightLift.setPower(0);
            } else if (smoothnessTimer.milliseconds() < 100) {
                liftTargetIN = LiftPositionIN;
                LeftLift.setPower(0);
                RightLift.setPower(0);
            } else {

                //LeftLift.setTargetPosition(LiftTarget);
                //RightLift.setTargetPosition(LiftTarget);

                liftPower = 0.5 * ((liftTargetIN == 0 ? -tolerance : liftTargetIN) - LiftPositionIN);
                leftLiftPower = 0.5 * ((liftTargetIN == 0 ? -tolerance : liftTargetIN + tilt * 2) - leftLiftPositionIN);
                rightLiftPower = 0.5 * ((liftTargetIN == 0 ? -tolerance : liftTargetIN - tilt * 2) - rightLiftPositionIN);

                if (Math.abs(liftPower) < 0.1) {
                    isBusy = false;
                    liftPower = 0;
                } else
                    isBusy = true;

//                liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (RobotConstants.LiftMotorTicksPerRotationofOuputShaft);

                LeftLift.setPower(leftLiftPower);
                RightLift.setPower(rightLiftPower);

            }

            if (slideOverride) {
                slideTargetIN = SlidePositionIN;
            } else {
                if (Math.abs(SlidePositionIN - slideTargetIN) < .15 || slideObstruction) {
                    Elbow.setPosition(0.5);
                    Elbow2.setPosition(0.5);
                } else {
                    Elbow.setPosition(Range.clip(Range.scale((slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8), 0.2, 0.8));
                    Elbow2.setPosition(Range.clip(Range.scale((slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8), 0.2, 0.8));
                    isBusy = true;
                }
            }
        }

    }

    public void updatePositions(RevBulkData bulkData2) {

        LiftPositionIN = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(RightLift)) / LiftTicksPerInch;
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;


        liftObstruction = SlidePositionIN > 1;
        slideObstruction = LiftPositionIN < 8;

    }
}
