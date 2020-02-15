package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class pwmtest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servo = (ServoImplEx) hardwareMap.servo.get("LeftAngle");
//        servo.setPwmRange(new PwmControl.PwmRange(500,2500,3003));
//        servo.setPosition(0.35);
        while (opModeIsActive()) {
            sleep(10000);
            servo.getController().pwmDisable();
            sleep(10000);
            servo.getController().pwmEnable();
        }
    }
}
