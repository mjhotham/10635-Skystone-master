package org.firstinspires.ftc.teamcode.worlds;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.State.LiftManager;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class GetLiftData extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        LiftManager lift = new LiftManager(drive.LeftLift, drive.RightLift, drive.Elbow, drive.Elbow2, drive.LeftIntake);
        waitForStart();
        lift.start(48);
        DataLogger log = new DataLogger("logos", "liftDataLog.csv", telemetry);

        log.addField("time");
        log.addField("liftPositionIn");
        log.addField("liftTargetIn");
//        log.addField("slidePositionIn");
        log.addField("voltage hub 1");
//        log.addField("voltage hub 2");

        log.addField("leftMotor EncPos");
        log.addField("leftMotor power");
        log.addField("leftMotor current");
        log.addField("leftMotor velocity");

        log.addField("rightMotor EncPos");
        log.addField("rightMotor power");
        log.addField("rightMotor velocity");
        log.addField("rightMotor current");

        log.newLine();

        while (lift.isBusy) {
            log.addField(getRuntime());
            log.addField(lift.LiftPositionIN);
            log.addField(lift.liftTargetIN);
//            log.addField(lift.SlidePositionIN);
            log.addField(drive.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));   // only need to know voltage of one attached to lift motors
//            log.addField(drive.hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));

            RevBulkData data = drive.hub2.getBulkInputData();

            log.addField(data.getMotorCurrentPosition(drive.LeftLift));
            log.addField(drive.LeftLift.getPower());
            log.addField(data.getMotorVelocity(drive.LeftLift));
            log.addField(drive.LeftLift.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

            log.addField(data.getMotorCurrentPosition(drive.RightLift));
            log.addField(drive.RightLift.getPower());
            log.addField(data.getMotorVelocity(drive.RightLift));
            log.addField(drive.RightLift.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

            log.newLine();
        }
        try {
            log.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }
    }
}
//not much