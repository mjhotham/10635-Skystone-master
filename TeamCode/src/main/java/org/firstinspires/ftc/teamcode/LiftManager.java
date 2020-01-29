package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

public class LiftManager {
    public ExpansionHubMotor LeftLift;
    public ExpansionHubMotor RightLift;
    public ExpansionHubMotor SlideEncoder;
    public ExpansionHubServo Elbow;
    public double liftTargetIN = 0;
    public double slideTargetIN = 0;
    public boolean isBusy = false;
    public double liftPower = 1;
    public double pidPower = 0.1;

    // Fixed ticks per inch (big oof, really sorry about that)
    public double LiftTicksPerInch = 537.6 / (1.25 * Math.PI);     // for gobilda 19.7:1 and 1.25 inch spool
    public double SlideTicksPerInch = 360 / (1.25 * Math.PI);      // for vex optical shaft encoder and 1.25 inch spool

    public double tolerance = 0.25;
    public double LiftPositionIN = 0;
    public double SlidePositionIN = 0;

    public boolean liftObstruction = false;
    public boolean slideObstruction = false;

    public LiftManager(ExpansionHubMotor LeftLift, ExpansionHubMotor RightLift, ExpansionHubServo Elbow, ExpansionHubMotor slideEncoder) {
        this.LeftLift = LeftLift;
        this.RightLift = RightLift;
        this.Elbow = Elbow;
        this.SlideEncoder = slideEncoder;
    }

    public void pause() {
        int target = Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition());
        LeftLift.setTargetPosition(target);
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift.setPower(pidPower);
        RightLift.setTargetPosition(target);
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift.setPower(pidPower);
        liftPower = pidPower;
        Elbow.setPosition(0.5);
    }

    public void stop() {
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void start() {
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftPositionIN = Math.PI * 1.25 * Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition()) / 537.6;
        liftTargetIN = LiftPositionIN;
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void start(double liftTargetIN) {
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftTargetIN = liftTargetIN;
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void update(RevBulkData bulkData2) {


        LiftPositionIN = Math.PI * 1.25 * Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition()) / 537.6;
        SlidePositionIN = Math.PI * 1.25 * bulkData2.getMotorCurrentPosition(SlideEncoder) / 360;

        liftObstruction = SlidePositionIN > 1;
        slideObstruction = LiftPositionIN < 8;

        if (liftObstruction && liftTargetIN <= 8 && LiftPositionIN < 8) {
            LeftLift.setPower(1);
            RightLift.setPower(1);
        } else if (liftObstruction && liftTargetIN <= 8 && LiftPositionIN < 10) {
            LeftLift.setPower(0);
            RightLift.setPower(0);
        } else {

            //LeftLift.setTargetPosition(LiftTarget);
            //RightLift.setTargetPosition(LiftTarget);


            double liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

            liftPower = (liftTargetIN - LiftPositionIN);

            LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
            RightLift.setPower(liftPower + Math.max(0, liftOffset));
        }

        if (Math.abs(LiftPositionIN - liftTargetIN) < tolerance) {
            isBusy = false;
            LeftLift.setPower(0);
            RightLift.setPower(0);
            liftPower = pidPower;
        } else
            isBusy = true;

        if (Math.abs(SlidePositionIN - slideTargetIN) < .15) {
            Elbow.setPosition(0.5);
        } else {
            if (slideObstruction)
                Elbow.setPosition(0.5);
            else
                Elbow.setPosition(Range.clip(
                        Range.scale(
                                (slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8),
                        0.2, 0.8));
            isBusy = true;
        }

    }

    ElapsedTime smoothnessTimer = new ElapsedTime();

    public void update(RevBulkData bulkData2, double triggerSum, boolean override, boolean slideOverride) {

        LiftPositionIN = Math.PI * 1.25 * Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition()) / 537.6;
        SlidePositionIN = Math.PI * 1.25 * bulkData2.getMotorCurrentPosition(SlideEncoder) / 360;

        liftObstruction = Math.abs(SlidePositionIN - slideTargetIN) > 2;
        slideObstruction = LiftPositionIN < 8;
//        int LiftTarget = (int) Math.round(liftTargetIN * LiftTicksPerInch);

        if (override) {
            Elbow.setPosition(0.5);
            if (Math.abs(triggerSum) > 0.1) {
                LeftLift.setPower(triggerSum);
                RightLift.setPower(triggerSum);
                liftTargetIN = LiftPositionIN;
            } else {
                RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                start();
            }

        } else {

            if (liftObstruction && liftTargetIN <= 8 && LiftPositionIN < 8) {
                double liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

                liftPower = 8.2 - LiftPositionIN;
                LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                RightLift.setPower(liftPower + Math.max(0, liftOffset));

            } else if (Math.abs(triggerSum) > 0.1 && (LiftPositionIN > 0 || triggerSum > 0)) {
                double liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

                liftPower = triggerSum;
                liftTargetIN = LiftPositionIN;

                smoothnessTimer.reset();

                LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                RightLift.setPower(liftPower + Math.max(0, liftOffset));
            } else if (liftObstruction && liftTargetIN <= 8 && LiftPositionIN < 10) {
                LeftLift.setPower(0);
                RightLift.setPower(0);
            } else if (smoothnessTimer.milliseconds() < 100) {
                liftTargetIN = LiftPositionIN;
                LeftLift.setPower(0);
                RightLift.setPower(0);
            } else {

                //LeftLift.setTargetPosition(LiftTarget);
                //RightLift.setTargetPosition(LiftTarget);

                liftPower = ((liftTargetIN == 0 ? -tolerance : liftTargetIN) - LiftPositionIN);

                if (Math.abs(LiftPositionIN - liftTargetIN) < tolerance) {
                    isBusy = false;
                    liftPower = 0;
                } else
                    isBusy = true;

                double liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

                LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
                RightLift.setPower(liftPower + Math.max(0, liftOffset));

            }

            if (slideOverride) {
                slideTargetIN = SlidePositionIN;
            } else {
                if (Math.abs(SlidePositionIN - slideTargetIN) < .15 || slideObstruction) {
                    Elbow.setPosition(0.5);
                } else {
                    Elbow.setPosition(Range.clip(
                            Range.scale(
                                    (slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8),
                            0.2, 0.8));
                    isBusy = true;
                }
            }
        }

    }

    public void updatePositions(RevBulkData bulkData2) {

        LiftPositionIN = Math.PI * 1.25 * Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition()) / 537.6;
        SlidePositionIN = Math.PI * 1.25 * bulkData2.getMotorCurrentPosition(SlideEncoder) / 360;

        liftObstruction = SlidePositionIN > 1;
        slideObstruction = LiftPositionIN < 8;

    }
}
