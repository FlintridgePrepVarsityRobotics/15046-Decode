package org.firstinspires.ftc.teamcode.auton;
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

@Autonomous(name = "AurixPPP Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class aurixp extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Shooting;
        public PathChain Spike2Alignment;
        public PathChain Spike2Intake;
        public PathChain GateOpen;
        public PathChain Scoring2;
        public PathChain Spike1Alignment;
        public PathChain Spike1Intake;
        public PathChain Scoring3;
        public PathChain Spike3;
        public PathChain Spike3Intake;
        public PathChain Scoring4;
        public PathChain Park;

        public Paths(Follower follower) {
            Shooting = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.500, 120.000),
                                    new Pose(104.000, 97.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(49))
                    .build();

            Spike2Alignment = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(104.000, 97.000),
                                    new Pose(89.000, 80.000),
                                    new Pose(100.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Spike2Intake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 60.000),
                                    new Pose(134.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            GateOpen = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.000, 60.000),
                                    new Pose(117.000, 58.000),
                                    new Pose(129.750, 64.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Scoring2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.750, 64.500),
                                    new Pose(74.000, 70.000),
                                    new Pose(104.000, 97.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
                    .build();

            Spike1Alignment = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(104.000, 97.000),
                                    new Pose(94.000, 90.000),
                                    new Pose(100.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Spike1Intake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 84.000),
                                    new Pose(127.500, 84.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Scoring3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.500, 84.000),
                                    new Pose(104.000, 97.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
                    .build();

            Spike3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(104.000, 97.000),
                                    new Pose(89.000, 66.500),
                                    new Pose(100.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))
                    .build();

            Spike3Intake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 36.000),
                                    new Pose(134.000, 36.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Scoring4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 36.000),
                                    new Pose(104.000, 97.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(104.000, 97.000),
                                    new Pose(110.000, 91.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(66))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return 0;
    }
}