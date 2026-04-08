package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class IntakeSubsystem {
    public static double ASlowDown = .9;
    public static double INTAKE_POWER = 1.0;
    public static double STALL_TIMEOUT_MS = 300.0;
    public static int STALL_TICKS_THRESHOLD = 0;

    private final DcMotorEx intake, transfer;
    private double lastIntakePos = 0;
    private ElapsedTime stallTimer = new ElapsedTime();

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setPower(0);
        transfer.setPower(0);
    }

    public void intakeFull() { intake.setPower(1); transfer.setPower(1.0); }
    public void intakeCustom() { intake.setPower(ASlowDown); transfer.setPower(ASlowDown); }
    public void intakeOff() { intake.setPower(0); transfer.setPower(0); }
    public void intakeReverse() { intake.setPower(-INTAKE_POWER); transfer.setPower(-INTAKE_POWER); }

    public boolean isStalled() {
        double currentPos = intake.getCurrentPosition();
        if (Math.abs(intake.getPower()) > 0.1) {
            if (Math.abs(currentPos - lastIntakePos) > STALL_TICKS_THRESHOLD) {
                stallTimer.reset();
                lastIntakePos = currentPos;
            }
            return stallTimer.milliseconds() > STALL_TIMEOUT_MS;
        }
        stallTimer.reset();
        return false;
    }
}