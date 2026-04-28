package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "FarBlue", group = "Autonomous")
@Configurable
public class FarBlue extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;

    ShooterSubsystem shooter;
    IntakeSubsystem intake;

    private int pathState = 0;
    private Paths paths;
    private ElapsedTime stateTimer = new ElapsedTime();

    public static double turretTargetDegrees = -81.0;

    enum ShotState { IDLE, OPEN_GATE, RUN_INTAKE, CLOSE_GATE, COOLDOWN }
    ShotState currentShotState = ShotState.IDLE;

    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime breakTimer = new ElapsedTime();
    boolean beamWasBroken = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(62.063, 8.248, Math.toRadians(180)));

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        shooter.closeGate();
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "FarBlue Initialized - All Cycles Restored");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.enableFlywheels();
        stateTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // Fixed Shooter Settings
        shooter.setTurretPosition(turretTargetDegrees);
        shooter.setFlywheelVelocity(1930, true);
        shooter.setHoodPosition(0.525);

        updateIntakeLogic();
        updateShotSequence();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shot State", currentShotState);
        panelsTelemetry.update(telemetry);
    }

    private void updateIntakeLogic() {
        boolean isCurrentlyBroken = intake.isFull();
        if (currentShotState == ShotState.IDLE) {
            if (isCurrentlyBroken) {
                if (!beamWasBroken) { breakTimer.reset(); beamWasBroken = true; }
                if (breakTimer.seconds() >= 0.3) intake.intakeOff();
            } else {
                intake.intakeFull();
                beamWasBroken = false;
                shooter.closeGate();
            }
        }
    }

    private void updateShotSequence() {
        switch (currentShotState) {
            case OPEN_GATE:
                if (shotTimer.seconds() >= 0.05) {
                    intake.farShot();
                    shotTimer.reset();
                    currentShotState = ShotState.RUN_INTAKE;
                }
                break;
            case RUN_INTAKE:
                if (shotTimer.seconds() >= 0.8) {
                    intake.intakeOff();
                    shooter.closeGate();
                    shotTimer.reset();
                    currentShotState = ShotState.COOLDOWN;
                }
                break;
            case COOLDOWN:
                if (shotTimer.seconds() >= 0.05) {
                    currentShotState = ShotState.IDLE;
                    beamWasBroken = false;
                }
                break;
        }
    }

    private void triggerShot() {
        if (currentShotState == ShotState.IDLE) {
            currentShotState = ShotState.OPEN_GATE;
            shooter.openGate();
            shotTimer.reset();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Preload
                follower.followPath(paths.ShootPreLoad);
                stateTimer.reset();
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy() || stateTimer.seconds() > 3.0) {
                    triggerShot();
                    pathState = 2;
                }
                break;

            case 2: // Spike 3
                if (currentShotState == ShotState.IDLE) {
                    turretTargetDegrees = -66.5;
                    follower.followPath(paths.IntakeSpike3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootSpike3);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    triggerShot();
                    pathState = 5;
                }
                break;

            case 5: // Cycle 1
                if (currentShotState == ShotState.IDLE) {
                    follower.followPath(paths.IntakeCycle3);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootCycle3);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    triggerShot();
                    pathState = 8;
                }
                break;

            case 8: // Cycle 2
                if (currentShotState == ShotState.IDLE) {
                    follower.followPath(paths.IntakeCycle2);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootCycle2);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    triggerShot();
                    pathState = 11;
                }
                break;

            case 11: // Cycle 3
                if (currentShotState == ShotState.IDLE) {
                    follower.followPath(paths.IntakeCycle3);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootCycle3);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    triggerShot();
                    pathState = 14;
                }
                break;

            case 14: // Leave
                if (currentShotState == ShotState.IDLE) {
                    follower.followPath(paths.Leave);
                    pathState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain ShootPreLoad, IntakeSpike3, ShootSpike3, IntakeCycle1, ShootCycle1, IntakeCycle2, ShootCycle2, IntakeCycle3, ShootCycle3, Leave;

        public Paths(Follower follower) {
            ShootPreLoad = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(62.063, 8.248), new Pose(58.925, 14.206)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();

            IntakeSpike3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(56.925, 17.206), new Pose(61.093, 42.981), new Pose(43.065, 42.355), new Pose(10.327, 35.037)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();

            ShootSpike3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(10.327, 35.037), new Pose(58.150, 14.346)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();

            IntakeCycle1 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(57.150, 17.346), new Pose(47.752, 11.701), new Pose(8.523, 9.421)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();

            ShootCycle1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(8.523, 9.421), new Pose(58.346, 14.056)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();

            IntakeCycle2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(57.346, 17.056), new Pose(48.112, 8.743), new Pose(8.869, 0.341), new Pose(9.009, 41.981)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(90))
                    .build();

            ShootCycle2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(9.009, 41.981), new Pose(58.065, 14.430)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(225))
                    .build();

            IntakeCycle3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(57.065, 17.430), new Pose(4.958, 42.495), new Pose(9.374, 7.813)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(270))
                    .build();

            ShootCycle3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(7.374, 8.813), new Pose(58.645, 14.822)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(57.645, 16.822), new Pose(51.757, 23.645)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(225))
                    .build();
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}