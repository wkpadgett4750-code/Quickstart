package org.firstinspires.ftc.teamcode.tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Configurable
@TeleOp(name="DashBoard Tuner")
public class DashboardTuner extends LinearOpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    public static boolean RUN_TRACKING = false;
    public static boolean USE_MANUAL_TICKS = false; // New toggle for static PID testing
    public static boolean RUN_FLYWHEELS = false;
    public static boolean RUN_INTAKE = false;

    private boolean lastA = false, lastB = false, lastX = false, lastY = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cp = follower.getPose();

            // 1. DRIVING
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double den = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(rx), 1);
            leftFront.setPower((driveY + driveX + rx) / den);
            leftBack.setPower((driveY - driveX + rx) / den);
            rightFront.setPower((driveY - driveX - rx) / den);
            rightBack.setPower((driveY + driveX - rx) / den);

            // 2. TOGGLES
            if (gamepad1.a && !lastA) RUN_TRACKING = !RUN_TRACKING;
            lastA = gamepad1.a;

            if (gamepad1.b && !lastB) RUN_INTAKE = !RUN_INTAKE;
            lastB = gamepad1.b;

            if (gamepad1.x && !lastX) RUN_FLYWHEELS = !RUN_FLYWHEELS;
            lastX = gamepad1.x;

            if (gamepad1.y && !lastY) USE_MANUAL_TICKS = !USE_MANUAL_TICKS;
            lastY = gamepad1.y;

            // 3. TURRET AIMING
            if (RUN_TRACKING) {
                shooter.trackingActive = true;
                if (USE_MANUAL_TICKS) {
                    // Manually move the turret to a specific tick count via Dashboard
                    shooter.setTurretPosition(ShooterSubsystem.tuningTurretTicks);
                } else {
                    // STATIC TRACKING: Passing 0 for vx and vy ignores "Shooting on the Move"
                    // This just points at the goal from where the robot is currently sitting.
                  //  shooter.alignTurret(cp.getX(), cp.getY(), cp.getHeading(), 0, 0, true, null, 0, false);
                }
            } else {
                shooter.stopTurret();
            }

            // 4. FLYWHEELS & HOOD
            if (RUN_FLYWHEELS) {
                shooter.setFlywheelVelocity(ShooterSubsystem.tuningRPM);
                shooter.setHoodPosition(ShooterSubsystem.tuningHoodPos);
            } else {
                shooter.disableFlywheels();
            }

            // 5. INTAKE
            if (RUN_INTAKE) intake.farShot();
            else intake.intakeOff();
// --- CALCULATE DISTANCE ---
            double distToGoal = Math.hypot(ShooterSubsystem.blueGoalX - cp.getX(), ShooterSubsystem.blueGoalY - cp.getY());
            // 6. TELEMETRY
            telemetry.addLine("--- SYSTEM STATUS ---");
            telemetry.addData("Mode", USE_MANUAL_TICKS ? "MANUAL TICKS" : "GOAL TRACKING");
            telemetry.addData("Distance from Goal", "%.2f in", distToGoal);

            telemetry.addLine("\n--- SHOOTER ---");
            telemetry.addData("Hood Position", ShooterSubsystem.tuningHoodPos);
            telemetry.addData("Target Velocity", ShooterSubsystem.tuningRPM);
            telemetry.addData("Actual Velocity", shooter.getCurrentVelocity());

            telemetry.addLine("\n--- TURRET ---");
            telemetry.addData("Turret Target Pos", USE_MANUAL_TICKS ? ShooterSubsystem.tuningTurretTicks : "AUTO-CALC");
            telemetry.addData("Turret Actual Pos", shooter.getTurretPos());

            telemetry.addLine("\n--- LOCALIZATION ---");
            telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, H: %.2f", cp.getX(), cp.getY(), Math.toDegrees(cp.getHeading()));

            telemetry.update();
        }
    }
}