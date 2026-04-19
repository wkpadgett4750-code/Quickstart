
package org.firstinspires.ftc.teamcode.autos;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "BottomBlue21", group = "Autonomous")
@Configurable // Panels
public class BottomBlue21 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    
    // Counter for the Gate cycle loop
    private int gateCycles = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(18.1, 119.5 , Math.toRadians(-36)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain PreLoadShot, IntakeSpike2, ShootSpike2, GateTake, GateShot, IntakeSpike1, ShootSpike1;
        public Paths(Follower follower) {
            PreLoadShot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.100, 119.500),
                                    new Pose(41.154, 101.158),
                                    new Pose(55.850, 85.907)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-135))
                    .build();

            IntakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.850, 85.907),
                                    new Pose(51.079, 58.463),
                                    new Pose(12.065, 63.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-180))

                    .build();

            ShootSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.065, 63.056),
                                    new Pose(50.290, 68.664),
                                    new Pose(56.290, 86.178)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-135))

                    .build();

            GateTake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.290, 86.178),
                                    new Pose(42.495, 69.005),
                                    new Pose(8.664, 59.439)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-210))

                    .build();

            GateShot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.664, 59.439),
                                    new Pose(39.500, 62.374),
                                    new Pose(55.981, 86.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-210), Math.toRadians(-135))

                    .build();

            IntakeSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.981, 86.131),

                                    new Pose(15.252, 85.458)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-155), Math.toRadians(-180))

                    .build();

            ShootSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.252, 85.458),

                                    new Pose(49.748, 115.673)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-135))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start by following the PreLoadShot path
                follower.followPath(paths.PreLoadShot);
                pathState = 1;
                break;

            case 1: // Wait until PreLoadShot is done, then Intake Spike 2
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeSpike2);
                    pathState = 2;
                }
                break;

            case 2: // Wait until Intake is done, then Shoot Spike 2
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootSpike2);
                    pathState = 3;
                }
                break;

            case 3: // Move to the Gate for Intake
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateTake);
                    pathState = 4;
                }
                break;

            case 4: // Gate Cycle End: Shoot from Gate
                if (!follower.isBusy()) {
                    gateCycles++; // Increment count after a full cycle
                    follower.followPath(paths.GateShot);

                    // If we haven't done 3 total cycles yet, go back to Case 3
                    if (gateCycles < 3) {
                        pathState = 3;
                    } else {
                        pathState = 5;
                    }
                }
                break;
            case 5: // Intake Spike 1
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeSpike1);
                    pathState = 6;
                }
                break;

            case 6: // Final Shoot for Spike 1
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootSpike1);
                    pathState = 7;
                }
                break;

            case 7: // Robot is finished
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Finished!");
                    pathState = -1; // End state
                }
                break;
        }
        return pathState;
    }
}
    