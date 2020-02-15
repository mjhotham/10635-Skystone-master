package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @TeleOp
public class getVuforiaNumbers extends LinearOpMode {
    private vuforiaLib vuforia = new vuforiaLib();
    public void runOpMode() {
        vuforia.init(this);
        vuforia.start();
        while(!isStopRequested()) {//-28.5, -21, -11.5
            telemetry.addData("output number", vuforia.getData(true));
            telemetry.update();
        }
    }
}
