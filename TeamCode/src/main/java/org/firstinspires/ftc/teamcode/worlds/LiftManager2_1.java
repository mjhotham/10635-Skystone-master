package org.firstinspires.ftc.teamcode.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

@Config
public class LiftManager2_1 {

    public ExpansionHubMotor LeftLift;
    public ExpansionHubMotor RightLift;
    public ExpansionHubMotor SlideEncoder;
    public ExpansionHubServo Elbow;
    public ExpansionHubServo Elbow2;
    public double liftTargetIN = 0;
    public double slideTargetIN = 0;
    public boolean isBusy = false;
    public double liftPower = 1;

    public double LiftTicksPerInch = RobotConstants.LiftMotorTicksPerRotationofOuputShaft / (RobotConstants.LiftSpoolDiameterIN * Math.PI);
    public double SlideTicksPerInch = RobotConstants.TopslideTicksPerRotation / (RobotConstants.TopslideSpoolDiameterIN * Math.PI);

    public double LiftPositionIN = 0;
    public double SlidePositionIN = 0;

    public boolean liftObstruction = false;
    public boolean slideObstruction = false;

    boolean PreviousSlideOverride = false;

    double leftLiftPositionIN;
    double rightLiftPositionIN;

    double leftLiftPower;
    double rightLiftPower;

    double SlidePower;

    double PreviousSlideTarget;

    boolean FriggenEdgeCase;


    //tuning this will be a process
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.2, 0, 0);          // starting point for tuning
    public static PIDCoefficients topSlideCoefficients = new PIDCoefficients(0.3, 0, 0);      // starting point for tuning

    public PIDFController leftLiftControl = new PIDFController(liftCoefficients);
    public PIDFController rightLiftControl = new PIDFController(liftCoefficients);
    public PIDFController topSlideControl = new PIDFController(topSlideCoefficients);


    public LiftManager2_1(ExpansionHubMotor LeftLift, ExpansionHubMotor RightLift, ExpansionHubServo Elbow, ExpansionHubServo Elbow2, ExpansionHubMotor slideEncoder) {
        this.LeftLift = LeftLift;
        this.RightLift = RightLift;
        this.Elbow = Elbow;
        this.Elbow2 = Elbow2;
        this.SlideEncoder = slideEncoder;
        leftLiftControl.setOutputBounds(-1, 1);
        rightLiftControl.setOutputBounds(-1, 1);
        topSlideControl.setOutputBounds(-1, 1);
    }

    public void pause(RevBulkData bulkData2) {
        double pidPower = 0.1;
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
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void start(RevBulkData bulkData2) {
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftPositionIN = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(RightLift)) / LiftTicksPerInch;
        liftTargetIN = LiftPositionIN;
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void start(double liftTargetIN) {
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.liftTargetIN = liftTargetIN;
        isBusy = true;
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }


    public void update(RevBulkData bulkData2) {
        update(bulkData2, 0, false, false, 0);
    }

    public void update(RevBulkData bulkData2, double triggerSum, boolean override, boolean slideOverride) {
        update(bulkData2, triggerSum, override, slideOverride, 0);
    }

    ElapsedTime smoothnessTimer = new ElapsedTime();

    public void update(RevBulkData bulkData2, double triggerSum, boolean override, boolean slideOverride, float tilt) {    // override should really be changed to encReset

        leftLiftPositionIN = bulkData2.getMotorCurrentPosition(LeftLift) / LiftTicksPerInch;
        rightLiftPositionIN = bulkData2.getMotorCurrentPosition(RightLift) / LiftTicksPerInch;

        LiftPositionIN = Math.max(leftLiftPositionIN, rightLiftPositionIN);
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;

        slideObstruction = LiftPositionIN < RobotConstants.TopSlideMinMovementHeight;

        liftObstruction = slideObstruction && SlidePositionIN > 2.5 && SlidePositionIN < 12;

//        liftObstruction = Math.abs(SlidePositionIN - slideTargetIN) > (slideObstruction ? 3 : 1);

        if (override) {

            if (Math.abs(triggerSum) > 0.1) {

                if (triggerSum < -.1 && !slideOverride) {
                    Elbow.setPosition(.5);
                    Elbow2.setPosition(.5);
                }

                LeftLift.setPower(triggerSum);
                RightLift.setPower(triggerSum);
                liftTargetIN = LiftPositionIN;

            } else {
                RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                start(bulkData2);
            }

        } else {

//            if(slideTargetIN < 12 && LiftPositionIN < RobotConstants.TopSlideMinMovementHeight && SlidePositionIN > 12){
//                slideObstruction = true;
//                liftTargetIN = 8;
//            } else if(slideTargetIN < 12 && LiftPositionIN > RobotConstants.TopSlideMinMovementHeight - .2 && SlidePositionIN < 1){
//                liftTargetIN = 0;
//            }

            if (Math.abs(triggerSum) > 0.1) {

                if (triggerSum < -.1 && !slideOverride) {
                    Elbow.setPosition(.5);
                    Elbow2.setPosition(.5);
                }

                LeftLift.setPower(triggerSum);
                RightLift.setPower(triggerSum);
                smoothnessTimer.reset();

            } else if (smoothnessTimer.milliseconds() < 100) {
                liftTargetIN = LiftPositionIN;
                LeftLift.setPower(0);
                RightLift.setPower(0);
            } else {

                leftLiftControl.setTargetPosition(liftObstruction ? 8 : liftTargetIN + tilt * 2);
                rightLiftControl.setTargetPosition(liftObstruction ? 8 : liftTargetIN - tilt * 2);

                leftLiftPower = leftLiftControl.update(leftLiftPositionIN);
                rightLiftPower = rightLiftControl.update(leftLiftPositionIN);

                liftPower = (leftLiftPower + rightLiftPower) / 2;

                if (Math.abs(liftPower) < 1) {
                    isBusy = false;
                } else
                    isBusy = true;

//                liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

                LeftLift.setPower(leftLiftPower);
                RightLift.setPower(rightLiftPower);

            }
        }


        if (slideOverride) {

            PreviousSlideOverride = true;
            slideTargetIN = SlidePositionIN;
            PreviousSlideTarget = slideTargetIN;
            topSlideControl.reset();  // havent tested with an i or d value so not sure if this is needed or not

        } else if (slideObstruction) {   // manual override should override slideobstruction for when shit hits the fan

            topSlideControl.setTargetPosition(SlidePositionIN);
            topSlideControl.update(SlidePositionIN);
            topSlideControl.reset();
            Elbow.setPosition(.5);
            Elbow2.setPosition(.5);

        } else if (PreviousSlideOverride) {

            Elbow.setPosition(.5);
            Elbow2.setPosition(.5);

            if (slideTargetIN != PreviousSlideTarget) {
                PreviousSlideOverride = false;

                if (triggerSum > -.1) {   // this code only runs for the first loop after a new target has been set, im not sure if it is even possible for it to run based on how notsure is but if does run, starts the lift one loop sooner

                    topSlideControl.setTargetPosition(slideTargetIN);
                    SlidePower = topSlideControl.update(SlidePositionIN);
                    Elbow.setPosition(Range.scale(SlidePower, -1, 1, 0.2, 0.8));
                    Elbow2.setPosition(Range.scale(SlidePower, -1, 1, 0.2, 0.8));
                    isBusy = Math.abs(SlidePositionIN - slideTargetIN) > .25;
                }
            }

        } else if (triggerSum > -.1) {
            topSlideControl.setTargetPosition(slideTargetIN);
            SlidePower = topSlideControl.update(SlidePositionIN);
            Elbow.setPosition(Range.scale(SlidePower, -1, 1, 0.2, 0.8));
            Elbow2.setPosition(Range.scale(SlidePower, -1, 1, 0.2, 0.8));
            isBusy = Math.abs(SlidePositionIN - slideTargetIN) > .25;
        }
    }

    public void updatePositions(RevBulkData bulkData2) {

        LiftPositionIN = Math.max(bulkData2.getMotorCurrentPosition(LeftLift), bulkData2.getMotorCurrentPosition(RightLift)) / LiftTicksPerInch;
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;

        liftObstruction = SlidePositionIN > 1;
        slideObstruction = LiftPositionIN < 8;

    }
}
