
package org.firstinspires.ftc.teamcode.auton; // make sure this aligns with class location


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name = "BlueberryAuto", group = "pedro")
public class BlueberryAuto extends OpMode {

    private TelemetryManager telemetryM;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;
    private final Pose startPose   = new Pose(72, 72, 0.5899212872 + Math.PI/2);

    private final Pose scorePose   = new Pose(80,  80, 0.5899212872 + Math.PI/2);


    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.


    private Path scorePreload;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:


           /* You could check for
           - Follower State: "if(!follower.isBusy()) {}"
           - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
           - Robot Position: "if(follower.getPose().getX() > 36) {}"
           */


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1,true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
        }
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        autonomousPathUpdate();
        follower.update();
        // Feedback to Driver Hub for debugging


        telemetryM.debug("Path State", pathState);
        telemetryM.debug("X", follower.getPose().getX());
        telemetryM.debug("Y", follower.getPose().getY());
        telemetryM.debug("Heading", follower.getPose().getHeading());
        telemetryM.update(telemetry);
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


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.update(telemetry);
        follower.update();
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        follower.update();
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.0
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.0
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
