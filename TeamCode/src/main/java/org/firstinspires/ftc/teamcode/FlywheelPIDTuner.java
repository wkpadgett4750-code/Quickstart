package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FLYWHEEL PIDF TUNER
 * ============================================================
 * PURPOSE: Tune kP, kI, kD, kF and RAMP_LIMIT for the flywheel.
 *
 * HOW TO USE:
 *   1. Open Panels on your device.
 *   2. Edit kP, kI, kD, kF, RAMP_LIMIT, TARGET_RPM live from the panel.
 *   3. Run this OpMode and watch telemetry for Current RPM vs Target RPM.
 *
 * TUNING PROCEDURE:
 *   Step 1 — kF first:
 *       Set kP=0, kI=0, kD=0. Increase kF until RPM is close but
 *       doesn't oscillate. Good starting guess: kF ≈ 1/maxRPM.
 *
 *   Step 2 — kP:
 *       Increase kP until the motor reaches setpoint quickly with
 *       minor overshoot. If it oscillates, back off.
 *
 *   Step 3 — kD (optional):
 *       Add small kD to dampen overshoot. Usually not needed if
 *       RAMP_LIMIT is doing its job.
 *
 *   Step 4 — RAMP_LIMIT:
 *       Controls max power increase per loop. Lower = smoother
 *       spin-up, higher = faster recovery after a shot.
 *
 *   Step 5 — kI (last resort):
 *       Only add kI if there's a persistent steady-state error.
 *       Keep very small (e.g. 0.0001). Windup will cause problems.
 *
 * CONTROLS:
 *   Gamepad1 DPAD_UP / DPAD_DOWN  → Target RPM +/- 50
 *   Gamepad1 A                    → Toggle flywheel on/off
 *   Gamepad1 B                    → Gate pulse (test shot recovery)
 *   Gamepad1 X                    → Reset PIDF to ShooterSubsystem defaults
 * ============================================================
 */
@Configurable
@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class FlywheelPIDTuner extends LinearOpMode {

    // --- Panels-editable constants ---
    public static double kP = ShooterSubsystem.kP;
    public static double kI = ShooterSubsystem.kI;
    public static double kD = ShooterSubsystem.kD;
    public static double kF = ShooterSubsystem.kF;
    public static double RAMP_LIMIT = ShooterSubsystem.RAMP_LIMIT;
    public static int TARGET_RPM = 2000;
    public static double RPM_TOLERANCE = ShooterSubsystem.rpmTolerance;

    private ShooterSubsystem shooter;
    private ElapsedTime gateTimer = new ElapsedTime();
    private boolean flywheelOn = false;
    private boolean lastA = false, lastB = false, lastX = false;
    private boolean lastUp = false, lastDown = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("Flywheel PIDF Tuner ready.");
        telemetry.addLine("Edit constants in Panels, then press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Push Panels values into subsystem each loop
            ShooterSubsystem.kP = kP;
            ShooterSubsystem.kI = kI;
            ShooterSubsystem.kD = kD;
            ShooterSubsystem.kF = kF;
            ShooterSubsystem.RAMP_LIMIT = RAMP_LIMIT;
            ShooterSubsystem.rpmTolerance = RPM_TOLERANCE;

            // Toggle flywheel
            if (gamepad1.a && !lastA) {
                flywheelOn = !flywheelOn;
                if (!flywheelOn) shooter.setFlywheelVelocity(0, false);
            }
            lastA = gamepad1.a;

            // Gate pulse — simulates a shot so you can watch RPM recovery
            if (gamepad1.b && !lastB) {
                shooter.openGate();
                gateTimer.reset();
            }
            lastB = gamepad1.b;
            if (gateTimer.seconds() > 0.3) shooter.closeGate();

            // Reset to ShooterSubsystem hardcoded defaults
            if (gamepad1.x && !lastX) {
                kP = 0.0096; kI = 0.0; kD = 0.0; kF = 0.00054; RAMP_LIMIT = 0.05;
            }
            lastX = gamepad1.x;

            // RPM stepping via gamepad
            if (gamepad1.dpad_up && !lastUp)
                TARGET_RPM = Math.min(TARGET_RPM + 50, (int) ShooterSubsystem.MAX_RPM);
            lastUp = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !lastDown)
                TARGET_RPM = Math.max(TARGET_RPM - 50, 0);
            lastDown = gamepad1.dpad_down;

            // Run flywheel
            if (flywheelOn) shooter.setFlywheelVelocity(TARGET_RPM, false);

            double currentRPM = shooter.getCurrentVelocity();
            double error = TARGET_RPM - currentRPM;

            // Telemetry
            telemetry.addLine("=== FLYWHEEL PIDF TUNER ===");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Current RPM", String.format("%.1f", currentRPM));
            telemetry.addData("Error", String.format("%.1f", error));
            telemetry.addData("At Speed", shooter.isAtSpeed() ? "YES" : "NO");
            telemetry.addLine("---");
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("RAMP_LIMIT", RAMP_LIMIT);
            telemetry.addLine("---");
            telemetry.addLine("A=Toggle  B=Gate  X=Reset  DPAD=RPM");
            telemetry.update();
        }

        shooter.setFlywheelVelocity(0, false);
    }
}