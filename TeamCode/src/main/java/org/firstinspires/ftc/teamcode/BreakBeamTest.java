package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BreakBeamTest")
public class BreakBeamTest extends LinearOpMode {

    private DigitalChannel breakBeam;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware mapping
        breakBeam = hardwareMap.get(DigitalChannel.class, "beam");
        breakBeam.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            boolean beamState = breakBeam.getState();

            if (beamState) {
                telemetry.addLine("Beam: GOOD (not broken)");
            } else {
                telemetry.addLine("Beam: BROKEN");
            }

            telemetry.update();
        }
    }
}