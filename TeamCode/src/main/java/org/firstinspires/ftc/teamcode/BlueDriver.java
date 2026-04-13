package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="BlueDriver", group="Linear Opmode")
public class BlueDriver extends LinearOpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    // Added WAIT_FOR_GATE to the state machine
    enum ShotState { IDLE, OPEN_GATE, WAIT_FOR_GATE, RUN_INTAKE, CLOSE_GATE }
    ShotState currentShotState = ShotState.IDLE;
    ElapsedTime shotTimer = new ElapsedTime();

    ElapsedTime breakTimer = new ElapsedTime();
    boolean beamWasBroken = false;
    boolean intakeAutoStopped = false;

    // Toggle variables for Right Trigger
    boolean intakeDisabledManually = false;
    boolean lastRT = false;

    private Pose startPose = new Pose(7.7, 8.1, Math.toRadians(0));
    private boolean lastLB = false;
    private boolean lastRB = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.closeGate();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            // 1. DRIVING
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            // 2. SHOOTER & TRIM
            if (gamepad1.a) { shooter.trackingActive = true; shooter.enableFlywheels(); }
            if (gamepad1.b) { shooter.trackingActive = false; shooter.disableFlywheels(); }

            if (gamepad1.left_bumper && !lastLB)  shooter.trimDegrees -= 1.0;
            if (gamepad1.right_bumper && !lastRB) shooter.trimDegrees += 1.0;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            shooter.alignTurret(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), true, telemetry, 0, false);

            // 3. INTAKE LOGIC (With Right Trigger Toggle)
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            if (rtPressed && !lastRT) {
                intakeDisabledManually = !intakeDisabledManually; // Flip the switch
            }
            lastRT = rtPressed;

            boolean isCurrentlyBroken = intake.isFull();
            if (currentShotState == ShotState.IDLE) {
                if (intakeDisabledManually) {
                    intake.intakeOff();
                }

                else if (isCurrentlyBroken) {
                    if (!beamWasBroken) {
                        breakTimer.reset();
                        beamWasBroken = true;
                    }

                    // Phase 1: Stop the intake after 0.4s of continuous blockage
                    if (breakTimer.seconds() >= 0.4 && !intakeAutoStopped) {
                        intake.intakeOff();
                        intakeAutoStopped = true;
                    }

                    // Phase 2: Wait an additional 0.5s AFTER the intake stops to open the gate
                    // (Total time from first break: 0.4s + 0.5s = 0.9s)
                    if (breakTimer.seconds() >= 0.9 && intakeAutoStopped) {
                        shooter.openGate();
                    }
                }
                else {
                    // Reset everything if the beam is cleared
                    intake.intakeFull();
                    beamWasBroken = false;
                    intakeAutoStopped = false;
                    // Optionally close the gate here if you want it to auto-shut when empty
                    // shooter.closeGate();
                }
            }

            // 4. SHOT SEQUENCE (Left Trigger Fire with .5s Delays)
            if (gamepad1.left_trigger > 0.1 && currentShotState == ShotState.IDLE) {
                currentShotState = ShotState.OPEN_GATE;
                shotTimer.reset();
            }

            switch (currentShotState) {
                case OPEN_GATE:
                    intake.intakeOff();
                    if (shotTimer.seconds() >= 0.5) {
                        shooter.openGate();
                        shotTimer.reset();
                        currentShotState = ShotState.WAIT_FOR_GATE;
                    }
                    break;

                case WAIT_FOR_GATE:
                    // Pause for .5s to let the gate fully move
                    if (shotTimer.seconds() >= 0.5) {
                        intake.intakeCustom(); // Turn on intake after delay
                        shotTimer.reset();
                        currentShotState = ShotState.RUN_INTAKE;
                    }
                    break;

                case RUN_INTAKE:
                    // Keep intake running for .5s to push ring through
                    if (shotTimer.seconds() >= 0.5) {
                        intake.intakeOff();
                        shotTimer.reset();
                        currentShotState = ShotState.CLOSE_GATE;
                    }
                    break;

                case CLOSE_GATE:
                    // Keep gate open for an extra .5s after intake stops as requested
                    if (shotTimer.seconds() >= 0.5) {
                        shooter.closeGate();
                        currentShotState = ShotState.IDLE;
                        intakeAutoStopped = false;
                        beamWasBroken = false;
                    }
                    break;
            }

            telemetry.addData("Intake System", intakeDisabledManually ? "DISABLED" : "AUTO");
            telemetry.addData("Shot Phase", currentShotState);
            telemetry.update();
        }
    }
}