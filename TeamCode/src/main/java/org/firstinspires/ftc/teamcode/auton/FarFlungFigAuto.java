package org.firstinspires.ftc.teamcode.auton; // make sure this aligns with class location


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "FarFlungFigAuto", group = "Autonomous")
@Configurable // Panels
public class FarFlungFigAuto extends OpMode {


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


        public PathChain preloadscore;
        public PathChain setup1;
        public PathChain pickup1;
        public PathChain shoot1;
        public PathChain setup2;
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain setup3;
        public PathChain pickup3;
        public PathChain shoot3;



        public Paths(Follower follower) {
            preloadscore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.457, 117.747), new Pose(44.865, 89.927))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(130))
                    .build();


                setup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.865, 89.927), new Pose(43.690, 83.853))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();


            pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.690, 83.853), new Pose(15.478, 84.049))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.478, 84.049), new Pose(44.865, 89.731))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();


            setup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.865, 89.731), new Pose(44.473, 59.559))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();


            pickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.473, 59.559), new Pose(15.086, 59.951))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.086, 59.951), new Pose(45.061, 89.731))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();


            setup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.061, 89.731), new Pose(43.298, 35.657))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();


            pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.298, 35.657), new Pose(15.478, 35.461))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.478, 35.461), new Pose(44.865, 89.535))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}



