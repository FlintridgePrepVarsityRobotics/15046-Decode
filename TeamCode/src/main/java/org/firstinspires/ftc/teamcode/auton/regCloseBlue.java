package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "regCloseBlue")
public class regCloseBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private final Pose startPose = new Pose(18.5, 120, Math.toRadians(143.5));

    private int pathState;

    public PathChain Shooting;
    public PathChain Spike2Alignment;
    public PathChain Spike2Intake;
    public PathChain GateOpen;
    public PathChain Scoring2;
    public PathChain Spike1Alignment;
    public PathChain Spike1Intake;
    public PathChain Scoring3;
    public PathChain Park;
    public void buildPaths() {

            Shooting = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.500, 120.000),

                                    new Pose(40.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(131))

                    .build();

            Spike2Alignment = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(40.000, 94.000),
                                    new Pose(55.000, 77.000),
                                    new Pose(44.000, 58.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Spike2Intake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 58.500),

                                    new Pose(14.000, 58.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            GateOpen = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 58.5),
                                    new Pose(27.000, 58.000),
                                    new Pose(14.250, 62)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Scoring2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.250, 61.500),
                                    new Pose(70.000, 67.000),
                                    new Pose(40.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))

                    .build();

            Spike1Alignment = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(40.000, 94.000),
                                    new Pose(50.000, 87.000),
                                    new Pose(44.000, 81.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Spike1Intake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 81.000),

                                    new Pose(16.500, 81.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Scoring3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.500, 81.000),

                                    new Pose(40.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 94.000),

                                    new Pose(34.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(114))

                    .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shooting);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Spike2Alignment,.5,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Spike2Intake,.5,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Spike2Intake,.5,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(GateOpen,.5,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Scoring2,.5,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Spike1Alignment,.5, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.followPath(Spike1Intake,.5, true);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.followPath(Scoring3,.5, true);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.followPath(Park,.5, true);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(10);
                }

                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }

                break;

        }
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}