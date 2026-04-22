package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="BlueDriver", group="Linear Opmode")
public class BlueDriver extends LinearOpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    enum ShotState { IDLE, OPEN_GATE, WAIT_FOR_GATE, RUN_INTAKE, CLOSE_GATE, COOLDOWN }
    ShotState currentShotState = ShotState.IDLE;
    ElapsedTime manualOnTimer = new ElapsedTime();
    ElapsedTime shotTimer = new ElapsedTime();

    ElapsedTime breakTimer = new ElapsedTime();
    boolean beamWasBroken = false;
    boolean intakeAutoStopped = false;

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

            // SOTM vector calculation removed, just grabbing position for standard tracking
            double distToGoal = Math.hypot(ShooterSubsystem.blueGoalX - currentPose.getX(), ShooterSubsystem.blueGoalY - currentPose.getY());

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

            // Updated alignTurret call (No velocity vectors needed)
            shooter.alignTurret(
                    currentPose.getX(),
                    currentPose.getY(),
                    currentPose.getHeading(),
                    true,
                    telemetry,
                    0,
                    false
            );

            // --- 3. INTAKE LOGIC ---
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            if (rtPressed && !lastRT) {
                intakeDisabledManually = !intakeDisabledManually;
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
                    if (breakTimer.seconds() >= 0.3) {
                        intake.intakeOff();
                        intakeAutoStopped = true;
                    }
                }
                else {
                    intake.intakeFull();
                    beamWasBroken = false;
                    intakeAutoStopped = false;
                    shooter.closeGate();
                }
            }

            // --- 4. SHOT SEQUENCE ---
            if (gamepad1.left_trigger > 0.5 && currentShotState == ShotState.IDLE) {
                currentShotState = ShotState.OPEN_GATE;
                shooter.openGate();
                shotTimer.reset();
            }

            switch (currentShotState) {
                case OPEN_GATE:
                    if (shotTimer.seconds() >= 0.15) {
                        // Distance-based shot intake speed injection
                        if (distToGoal > 140) {
                            intake.farShot();
                        } else {
                            intake.closeShot();
                        }

                        shotTimer.reset();
                        currentShotState = ShotState.RUN_INTAKE;
                    }
                    break;

                case RUN_INTAKE:
                    if (shotTimer.seconds() >= 0.6) {
                        intake.intakeOff();
                        shooter.closeGate();
                        shotTimer.reset();
                        currentShotState = ShotState.COOLDOWN;
                    }
                    break;

                case COOLDOWN:
                    if (shotTimer.seconds() >= 0.1) {
                        currentShotState = ShotState.IDLE;
                        intakeAutoStopped = false;
                        beamWasBroken = false;
                    }
                    break;
            }

            telemetry.addData("Dist to Goal", distToGoal);
            telemetry.addData("Intake System", intakeDisabledManually ? "DISABLED" : "AUTO");
            telemetry.addData("Shot Phase", currentShotState);
            telemetry.update();
        }
    }
}