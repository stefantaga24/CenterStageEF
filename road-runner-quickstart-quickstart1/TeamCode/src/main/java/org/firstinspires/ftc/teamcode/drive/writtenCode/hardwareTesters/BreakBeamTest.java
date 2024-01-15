package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BreakBeamTest", group = "Linear Opmode")
//s0Ft ArabEsc
public class BreakBeamTest extends LinearOpMode {
    DigitalChannel sensor;

    public void runOpMode() {
        sensor = hardwareMap.digitalChannel.get("beamfront");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
            telemetry.addData("state", ":  " + sensor.getState());
            telemetry.update();
        }
    }

}