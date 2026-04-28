package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="RedDriver", group="Linear Opmode")
public class RedDriver extends LinearOpMode {

    // Controller State Tracking
    private boolean autoAlignEnabled = true;
    private boolean lastY1 = false;          // For the GP1 Y Toggle
    private double manualTargetDegrees = 0;  // Stores the angle when locked

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    enum ShotState { IDLE, OPEN_GATE, WAIT_FOR_GATE, RUN_INTAKE, CLOSE_GATE, COOLDOWN }
    ShotState currentShotState = ShotState.IDLE;
    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime breakTimer = new ElapsedTime();

    boolean beamWasBroken = false;
    boolean intakeAutoStopped = false;
    boolean intakeDisabledManually = false;
    boolean lastRT = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
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
        follower.setStartingPose(PoseStorage.currentPose);
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            // Distance to Red Goal (144, 144)
            double distToGoal = Math.hypot(ShooterSubsystem.redGoalX - currentPose.getX(), ShooterSubsystem.redGoalY - currentPose.getY());

            // 1. CHASSIS DRIVE (GP1)
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x * 1.1;
            double driveRX = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(driveRX), 1);

            leftFront.setPower((driveY + driveX + driveRX) / denominator);
            leftBack.setPower((driveY - driveX + driveRX) / denominator);
            rightFront.setPower((driveY - driveX - driveRX) / denominator);
            rightBack.setPower((driveY + driveX - driveRX) / denominator);

            // 2. TURRET LOCK TOGGLE (GP1 Y)
            if (gamepad1.y && !lastY1) {
                autoAlignEnabled = !autoAlignEnabled;
                if (!autoAlignEnabled) {
                    // Capture turret angle at the moment of the click
                    manualTargetDegrees = shooter.getTurretAngle();
                }
            }
            lastY1 = gamepad1.y;

            // 3. SHOOTER EXECUTION (RED TARGETING)
            if (autoAlignEnabled) {
                // TRACKING MODE: Red Goal = false
                shooter.alignTurret(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), false, telemetry, 0, false);
            } else {
                // LOCK MODE: Flywheels/Hood still update based on distance
                double targetRPM = shooter.interpolate(distToGoal, ShooterSubsystem.distances, ShooterSubsystem.rpms);
                double targetHood = shooter.interpolate(distToGoal, ShooterSubsystem.distances, ShooterSubsystem.hoods);

                shooter.setFlywheelVelocity(targetRPM);
                shooter.setHoodPosition(targetHood);

                // Hold the captured angle
                shooter.setTurretPosition(manualTargetDegrees);
            }

            // Global Flywheel Enable/Kill (GP1)
            if (gamepad1.a) { shooter.trackingActive = true; shooter.enableFlywheels(); }
            if (gamepad1.b) { shooter.trackingActive = false; shooter.disableFlywheels(); }

            // 4. RECALIBRATION (Combo: GP2 Y + GP1 Dpad Up)
            if (gamepad2.y && gamepad1.dpad_up) {
                follower.setPose(new Pose(8.0, 8.0, Math.toRadians(180)));
                gamepad1.rumble(300);
            }

            // 5. TRIM CONTROLS (GP1 Bumpers)
            if (gamepad1.left_bumper && !lastLB)  shooter.trimDegrees -= 1.0;
            if (gamepad1.right_bumper && !lastRB) shooter.trimDegrees += 1.0;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            // 6. INTAKE & SHOT LOGIC
            updateShotLogic(distToGoal);

            // 7. TELEMETRY
            telemetry.addData("TEAM", "RED");
            telemetry.addData("TURRET MODE", autoAlignEnabled ? "AUTO-TRACKING" : "LOCKED AT " + Math.round(manualTargetDegrees) + "°");
            telemetry.addData("Dist to Goal", distToGoal);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.update();
        }
    }

    private void updateShotLogic(double distToGoal) {
        boolean rtPressed = gamepad1.right_trigger > 0.5;
        if (rtPressed && !lastRT) intakeDisabledManually = !intakeDisabledManually;
        lastRT = rtPressed;

        boolean isCurrentlyBroken = intake.isFull();

        if (currentShotState == ShotState.IDLE) {
            if (intakeDisabledManually) {
                intake.intakeOff();
            } else if (isCurrentlyBroken) {
                if (!beamWasBroken) { breakTimer.reset(); beamWasBroken = true; }
                if (breakTimer.seconds() >= 0.3) { intake.intakeOff(); intakeAutoStopped = true; }
            } else {
                intake.intakeFull();
                beamWasBroken = false;
                shooter.closeGate();
            }
        }

        if (gamepad1.left_trigger > 0.5 && currentShotState == ShotState.IDLE) {
            currentShotState = ShotState.OPEN_GATE;
            shooter.openGate();
            shotTimer.reset();
        }

        switch (currentShotState) {
            case OPEN_GATE:
                if (shotTimer.seconds() >= 0.15) {
                    if (distToGoal > 140) intake.farShot(); else intake.closeShot();
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
                    beamWasBroken = false;
                }
                break;
        }
    }
}