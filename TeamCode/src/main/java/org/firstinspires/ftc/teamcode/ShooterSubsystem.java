package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {
    // Hardware
    private DcMotorEx shooterLeft, shooterRight, rf;
    private CRServo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;

    // --- TUNED TURRET CONSTANTS (From TurretMathTracker) ---
    public static double blueGoalX = 0.0, blueGoalY = 144.0;
    public static double redGoalX = 0.0, redGoalY = -144.0;
    public static double angleOffset = -90.0;
    public static double tSlope = -67.9055;
    public static double turretOffsetX = 1.5;
    public static double turretOffsetY = 0.0;
    public static int turretMin = -8500, turretMax = 13000;
    public static double ticksPerRev = Math.abs(tSlope * 360.0);
    public static double tuningRPM = 1500;
    public static double tuningHoodPos = 0.5;
    public static int tuningTurretTicks = 0;

    // PIDF Constants
    public static double tp = 0.00011, ti = 0, td = 0.0002, ts = 0.061;
    public static int deadband = 10;

    // --- FLYWHEEL & HOOD CONSTANTS ---
    public static double MAX_RPM = 2800, RAMP_LIMIT = 0.05, rpmTolerance = 50;
    public static double kP = 0.001, kI = 0, kD = 0, kF = 0.00057;
    public static double sSlope = 6.30776, sIntercept = 906.54021;
    public static double hSlope = -0.00131286, hIntercept = 0.708604;
    public static double hoodMinPos = 0, hoodMaxPos = .7;
    public static double gateOpenPos = 0, gateClosedPos = .95;

    // State Variables
    public boolean trackingActive = false;
    public boolean flywheelsEnabled = false;
    public double trimDegrees = 0.0;
    private double currentTargetVelocity = 0, lastPower = 0;
    private double integralSum = 0, lastError = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.turretLeft = hardwareMap.get(CRServo.class, "turret1");
        this.turretRight = hardwareMap.get(CRServo.class, "turret2");
        this.rf = hardwareMap.get(DcMotorEx.class, "RF");
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        this.shooterHood = hardwareMap.get(Servo.class, "hood");
        this.shooterGate = hardwareMap.get(Servo.class, "gate");

        this.shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.turretLeft.setDirection(CRServo.Direction.REVERSE);
        this.turretRight.setDirection(CRServo.Direction.REVERSE);

        this.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Replaced with logic from TurretMathTracker
     */
    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double driverOffset, boolean isAuto) {
        if (!trackingActive && !isAuto) {
            stopTurret();
            if (!flywheelsEnabled) setFlywheelVelocity(0, isAuto);
            return;
        }

        // 1. Distance & Regression Logic (Distance remains standard hypot)
        double dist = distToGoal(x, y, blue);
        if (flywheelsEnabled || isAuto) {
            setFlywheelVelocity(getRegressionRPM(dist), isAuto);
            setHoodPosition(getRegressionHood(dist));
        }

        // 2. Advanced World-Locking Math (From Working Program)
        double targetX = blue ? blueGoalX : redGoalX;
        double targetY = blue ? blueGoalY : redGoalY;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Offset adjustment accounting for turret position relative to robot center
        double adjX = x + (turretOffsetX * cos - turretOffsetY * sin);
        double adjY = y + (turretOffsetX * sin + turretOffsetY * cos);

        double angleToGoal = Math.toDegrees(Math.atan2(targetX - adjX, targetY - adjY));

        // Target calculation including the Heading, Offset, and Trim
        double targetRel = angleToGoal + Math.toDegrees(heading) - angleOffset + trimDegrees + driverOffset;

        // 3. Robust Wrapping & Continuous Logic
        while (targetRel > 180)  targetRel -= 360;
        while (targetRel < -180) targetRel += 360;

        int targetTicks = (int) (targetRel * tSlope);

        // Continuous Wrap Check
        if (targetTicks < turretMin) {
            int alt = (int) (targetTicks + (int)ticksPerRev);
            if (alt <= turretMax) targetTicks = alt;
        } else if (targetTicks > turretMax) {
            int alt = (int) (targetTicks - (int)ticksPerRev);
            if (alt >= turretMin) targetTicks = alt;
        }

        // 4. Execution via PIDF
        setTurretPosition(targetTicks);

        if (telemetry != null) {
            telemetry.addData("Turret Tgt Ticks", targetTicks);
            telemetry.addData("Turret Trim", trimDegrees);
        }
    }

    public void setTurretPosition(int targetTicks) {
        int currentTicks = rf.getCurrentPosition();
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        double error = safeTarget - currentTicks;

        if (Math.abs(error) <= deadband) {
            turretLeft.setPower(0);
            turretRight.setPower(0);
            integralSum = 0;
            return;
        }

        integralSum += error;
        double derivative = error - lastError;
        double power = (error * tp) + (integralSum * ti) + (derivative * td) + Math.copySign(ts, error);
        lastError = error;

        double finalPower = Range.clip(power, -0.9, 0.9);
        turretLeft.setPower(finalPower);
        turretRight.setPower(finalPower);
    }

    public void setFlywheelVelocity(double targetRPM, boolean isAuto) {
        targetRPM = Math.min(targetRPM, MAX_RPM);
        this.currentTargetVelocity = targetRPM;
        if (targetRPM <= 0) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            lastPower = 0;
            return;
        }

        if (isAuto) {
            shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterLeft.setVelocity(targetRPM);
            shooterRight.setVelocity(targetRPM);
        } else {
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentVel = shooterLeft.getVelocity();
            double pidfPower = (targetRPM - currentVel) * kP + (targetRPM * kF);
            if (pidfPower > lastPower + RAMP_LIMIT) pidfPower = lastPower + RAMP_LIMIT;
            double finalPower = Range.clip(pidfPower, 0, 1.0);
            shooterLeft.setPower(finalPower);
            shooterRight.setPower(finalPower);
            lastPower = finalPower;
        }
    }

    public void stopTurret() {
        turretLeft.setPower(0);
        turretRight.setPower(0);
        integralSum = 0;
    }

    public void setHoodPosition(double pos) {
        shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos));
    }

    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }
    public void enableFlywheels() { flywheelsEnabled = true; }
    public void disableFlywheels() { flywheelsEnabled = false; setFlywheelVelocity(0, false); }
    public double distToGoal(double x, double y, boolean blue) {
        return Math.hypot((blue ? blueGoalX : redGoalX) - x, (blue ? blueGoalY : redGoalY) - y);}
    public int getRegressionRPM(double d) { return (int) (sSlope * d + sIntercept); }
    public double getRegressionHood(double d) { return Range.clip(hSlope * d + hIntercept, hoodMinPos, hoodMaxPos); }
    public double getCurrentVelocity() {return shooterLeft.getVelocity();}
    public int getTurretPos() {return rf.getCurrentPosition();}
    public boolean isAtSpeed() {return Math.abs(getCurrentVelocity() - currentTargetVelocity) < rpmTolerance;}
}