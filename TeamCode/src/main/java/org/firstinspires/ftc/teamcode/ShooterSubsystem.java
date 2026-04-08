package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ShooterSubsystem {

    private DcMotorEx shooterLeft, shooterRight;

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00057;

    private double lastPower = 0;
    private final double RAMP_LIMIT = 0.02;
    private final double MAX_RPM = 6000;

    private double currentTargetVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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

            // FTC expects ticks/sec, not RPM
            double ticksPerSecond = (targetRPM / 60.0) * shooterLeft.getMotorType().getTicksPerRev();

            shooterLeft.setVelocity(ticksPerSecond);
            shooterRight.setVelocity(ticksPerSecond);

        } else {
            shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double currentVel = shooterLeft.getVelocity(); // ticks/sec
            double currentRPM = (currentVel / shooterLeft.getMotorType().getTicksPerRev()) * 60.0;

            double error = targetRPM - currentRPM;

            // Manual PIDF
            double pidfPower = (kP * error) + (kF * targetRPM);

            // Ramp limit
            if (pidfPower > lastPower + RAMP_LIMIT)
                pidfPower = lastPower + RAMP_LIMIT;

            double finalPower = Range.clip(pidfPower, 0, 1.0);

            shooterLeft.setPower(finalPower);
            shooterRight.setPower(finalPower);

            lastPower = finalPower;
        }
    }

    public double getCurrentVelocity() {
        return shooterLeft.getVelocity();
    }
    public double getTargetVelocity() {
        return currentTargetVelocity;
    }

}