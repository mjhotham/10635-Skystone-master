package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @TeleOp
public class getVuforiaNumbers2 extends LinearOpMode {
//    private vuforiaLib2 vuforia = new vuforiaLib2();
    public void runOpMode() {
//        vuforia.init(this);
//        vuforia.start();
        while(!isStopRequested()) {//-28.5, -21, -11.5
//            telemetry.addData("output number", vuforia.getData(true));
            telemetry.update();
        }
    }
}
