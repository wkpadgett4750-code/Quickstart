package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {
    // Hardware
    private DcMotorEx shooterLeft, shooterRight, rf;
    private CRServo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;
    private VoltageSensor batteryVoltageSensor;

    // --- UPDATED LOOK-UP TABLE (LUT) DATA ---
    // Sorted from closest to furthest
    public static double[] distances = {42.08, 53.03, 64.34, 67.62, 69.52, 74.42, 84.0, 93.5, 102.94, 113.7, 130, 150.86, 154.53, 160.19};
    public static double[] rpms      = {1290, 1300, 1470, 1540, 1530, 1600, 1660, 1700, 1740, 1770, 1800, 1950, 1960, 1980};
    public static double[] hoods     = {0.70, 0.645, 0.585, 0.56, 0.55, 0.53, 0.525, 0.515, 0.51, 0.505, .505, 0.52, 0.52, 0.52};

    // --- TURRET CONSTANTS ---
    public static double blueGoalX = 0.0, blueGoalY = 144.0;
    public static double redGoalX = 0.0, redGoalY = -144.0;
    public static double angleOffset = -90.0;
    public static double tSlope = -67.9055;
    public static double turretOffsetX = 1.5;
    public static double turretOffsetY = 0.0;
    public static int turretMin = -8600, turretMax = 9000;
    public static double ticksPerRev = Math.abs(tSlope * 360.0);

    // Dashboard Tuning
    public static double tuningRPM = 1500, tuningHoodPos = 0.5;
    public static int tuningTurretTicks = 0;

    // --- TUNED TURRET PID VALUES ---
    public static double tp = 0.0001;
    public static double ti = 0.000004;
    public static double td = 0.0007;
    public static double ts = 0.074;
    public static int deadband = 15;

    // Flywheel Constants
    public static double MAX_RPM = 2000, RAMP_LIMIT = 0.05, rpmTolerance = 50;
    public static double kP = 0.001, kF = 0.000427;
    public static double hoodMinPos = 0, hoodMaxPos = .7;
    public static double gateOpenPos = .67, gateClosedPos = .52;
    public static double globalRPMTrim = 0.0;

    // State
    public boolean trackingActive = false, flywheelsEnabled = false;
    public double trimDegrees = 0.0;
    private double currentTargetVelocity = 0;
    private double lastFlywheelPower = 0;
    private double lastTurretPower = 0;
    private double lastTurretError = 0;
    private double turretIntegralSum = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.turretLeft = hardwareMap.get(CRServo.class, "turret1");
        this.turretRight = hardwareMap.get(CRServo.class, "turret2");
        this.rf = hardwareMap.get(DcMotorEx.class, "RF");
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        this.shooterHood = hardwareMap.get(Servo.class, "hood");
        this.shooterGate = hardwareMap.get(Servo.class, "gate");
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.turretLeft.setDirection(CRServo.Direction.REVERSE);
        this.turretRight.setDirection(CRServo.Direction.REVERSE);
        this.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Static Odometry Goal Tracking (No SOTM)
     */
    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double driverOffset, boolean isAuto) {
        if (!trackingActive && !isAuto) {
            stopTurret();
            return;
        }

        double targetX = blue ? blueGoalX : redGoalX;
        double targetY = blue ? blueGoalY : redGoalY;

        double distToRealGoal = Math.hypot(targetX - x, targetY - y);

        if (flywheelsEnabled || isAuto) {
            setFlywheelVelocity(interpolate(distToRealGoal, distances, rpms), isAuto);
            setHoodPosition(interpolate(distToRealGoal, distances, hoods));
        }

        // World-Locking Turret Math (Static)
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double adjX = x + (turretOffsetX * cos - turretOffsetY * sin);
        double adjY = y + (turretOffsetX * sin + turretOffsetY * cos);

        double angleToGoal = Math.toDegrees(Math.atan2(targetX - adjX, targetY - adjY));
        double targetRel = angleToGoal + Math.toDegrees(heading) - angleOffset + trimDegrees + driverOffset;

        while (targetRel > 180)  targetRel -= 360;
        while (targetRel < -180) targetRel += 360;
        int targetTicks = (int) (targetRel * tSlope);

        if (targetTicks < turretMin) {
            if (targetTicks + (int)ticksPerRev <= turretMax) targetTicks += (int)ticksPerRev;
        } else if (targetTicks > turretMax) {
            if (targetTicks - (int)ticksPerRev >= turretMin) targetTicks -= (int)ticksPerRev;
        }

        setTurretPosition(targetTicks);

        if (telemetry != null) {
            telemetry.addData("Dist", distToRealGoal);
        }
    }

    public double interpolate(double target, double[] xPoints, double[] yPoints) {
        if (target <= xPoints[0]) return yPoints[0];
        if (target >= xPoints[xPoints.length - 1]) return yPoints[yPoints.length - 1];

        for (int i = 0; i < xPoints.length - 1; i++) {
            if (target >= xPoints[i] && target <= xPoints[i + 1]) {
                double fraction = (target - xPoints[i]) / (xPoints[i + 1] - xPoints[i]);
                return yPoints[i] + (yPoints[i + 1] - yPoints[i]) * fraction;
            }
        }
        return yPoints[0];
    }

    public void setTurretPosition(int targetTicks) {
        int currentTicks = rf.getCurrentPosition();
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        double error = safeTarget - currentTicks;

        if (Math.abs(error) <= deadband) {
            stopTurret();
            return;
        }

        if (Math.abs(error) < 400) {
            turretIntegralSum += error;
        } else {
            turretIntegralSum = 0;
        }

        double derivative = (error - lastTurretError);
        double feedForward = (Math.abs(error) > 80) ? Math.copySign(ts, error) : (error / 80.0) * ts;
        double power = (error * tp) + (turretIntegralSum * ti) + (derivative * td) + feedForward;

        lastTurretError = error;
        lastTurretPower = Range.clip(power, -0.9, 0.9);
        turretLeft.setPower(lastTurretPower);
        turretRight.setPower(lastTurretPower);
    }

    public void setFlywheelVelocity(double targetRPM) {
        setFlywheelVelocity(targetRPM, false);
    }

    public void setFlywheelVelocity(double targetRPM, boolean isAuto) {
        targetRPM += globalRPMTrim;
        targetRPM = Math.min(targetRPM, MAX_RPM);
        this.currentTargetVelocity = targetRPM;

        if (targetRPM <= 0) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            lastFlywheelPower = 0;
            return;
        }

        double voltage = 12.0;
        try { voltage = batteryVoltageSensor.getVoltage(); } catch (Exception e) {}

        double compensatedFF = (targetRPM * kF) * (12.0 / voltage);
        double error = targetRPM - shooterLeft.getVelocity();
        double requestedPower = compensatedFF + (error * kP);

        if (requestedPower > lastFlywheelPower + RAMP_LIMIT) {
            requestedPower = lastFlywheelPower + RAMP_LIMIT;
        }

        double finalPower = Range.clip(requestedPower, 0, 1.0);
        shooterLeft.setPower(finalPower);
        shooterRight.setPower(finalPower);
        lastFlywheelPower = finalPower;
    }

    public void setHoodPosition(double pos) { shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos)); }
    public void stopTurret() { turretLeft.setPower(0); turretRight.setPower(0); turretIntegralSum = 0; }
    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }
    public void enableFlywheels() { flywheelsEnabled = true; }
    public void disableFlywheels() { flywheelsEnabled = false; setFlywheelVelocity(0, false); }
    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public int getTurretPos() { return rf.getCurrentPosition(); }
    public boolean isAtSpeed() { return Math.abs(getCurrentVelocity() - currentTargetVelocity) < rpmTolerance; }
}