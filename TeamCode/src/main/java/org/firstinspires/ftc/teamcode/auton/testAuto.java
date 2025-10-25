package org.firstinspires.ftc.teamcode.auton; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "BlueberryAuto", group = "fruitauto")
public class testAuto extends OpMode {

    public HWMap robot = new HWMap();
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(16.06530612244898, 118.92244897959183, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(38.4, 96.39183673469388, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(44.27755102040816, 79, Math.toRadians(180));
    private final Pose pickupintakePose = new Pose(28.367, 84.24489795918367, Math.toRadians(180));
    private Path scorePreload;
    private boolean returningToScore = false;

    private PathChain grabPickup1, intakePickup1, scorePickup1, scorePickup2, grabPickup3, scorePickup3;
    public static double kP = 0.0003;
    public static double kI = 0.0006;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private boolean shooting = false;
    private int shootStage = 0;
    private int shotsFired = 0;
    private double targetRPM = 2125;
    private final int ticksPerRevLauncher = 28;

    public void startShooting3() {
        shooting = true;
        shootStage = 0;
        shotsFired = 0;
        actionTimer.resetTimer();
    }
    public void updateShooting() {
        if (!shooting) return;

        double targetTicksPerSec = targetRPM / 60.0 * ticksPerRevLauncher;
        double measuredTicksPerSec = robot.launcher.getVelocity();
        double ffOutput = feedforward.calculate(targetTicksPerSec);
        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);
        double combinedOutput = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
        robot.launcher.setPower(combinedOutput);

        double errorRPM = Math.abs(measuredTicksPerSec / ticksPerRevLauncher * 60.0 - targetRPM);

        switch (shootStage) {
            case 0:
                // spin up until within tolerance OR 4 s max
                if (errorRPM < 75 || actionTimer.getElapsedTimeSeconds() > 4.0) {
                    shootStage = 1;
                    actionTimer.resetTimer();
                }
                break;

            case 1:
                // feed one ring
                robot.intake.setPower(0.6);
                robot.intakeServo.setPower(1);
                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(0);
                    shootStage = 2;
                    actionTimer.resetTimer();
                    shotsFired++;
                }
                break;

            case 2:
                // wait until RPM recovers instead of fixed time
                if (errorRPM < 50 && actionTimer.getElapsedTimeSeconds() > 0.3) {
                    if (shotsFired < 3) {
                        shootStage = 1;
                        actionTimer.resetTimer();
                    } else {
                        shootStage = 3;
                    }
                }
                break;

            case 3:
                robot.launcher.setPower(0);
                shooting = false;
                break;
        }
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setTranslationalConstraint(0.001);
        scorePreload.setHeadingConstraint(.001);
        scorePreload.setHeadingConstraint(.001);

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickupintakePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickupintakePose.getHeading())
                .setTranslationalConstraint(0.3)  // lower = slower (e.g., 0.3 m/s)
                .setHeadingConstraint(0.5)        // optional slower turn rate
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupintakePose, scorePose))
                .setLinearHeadingInterpolation(pickupintakePose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                // Wait until drive path is complete and heading is stable before firing
                if (!follower.isBusy()) {
                    double currentHeading = follower.getPose().getHeading();
                    double headingError = Math.abs(currentHeading - scorePose.getHeading());

                    // only start shooting once robot is aligned within ~5 degrees
                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
                        startShooting3();
                    }

                    // move on only after shooting finishes
                    if (!shooting && shotsFired >= 3) {
                        targetRPM = 0;
                        if (!shooting && !follower.isBusy()) {
                            follower.followPath(grabPickup1, true);
                        }
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    double currentRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);

                    // Start intake once launcher stopped
                    if (currentRPM <= 100 && !robot.intake.isBusy()) {
                        robot.intake.setPower(0.65);
                        robot.intakeServo.setPower(1);

                        follower.followPath(intakePickup1, true);
                        actionTimer.resetTimer();
                        setPathState(21);  // temporary sub-state for intake monitoring
                    }
                }
                break;

            case 21: // after intake finishes, do reverse launcher pulse
                if (!follower.isBusy()) {
                    // short reverse pulse (~100 ms)
                    if (actionTimer.getElapsedTimeSeconds() < 0.25) {
                        robot.launcher.setPower(-0.2);
                        robot.intake.setPower(-0.1);
                    } else {
                        robot.launcher.setPower(0);
                        robot.intake.setPower(0);
                        robot.intakeServo.setPower(0);
                        setPathState(3);  // proceed to next action
                    }
                }
                break;

            case 3:
                if (!returningToScore) {
                    follower.followPath(scorePickup1, true);
                    returningToScore = true;
                }

                if (!follower.isBusy()) {
                    double currentHeading = follower.getPose().getHeading();
                    double headingError = Math.abs(currentHeading - scorePose.getHeading());

                    // Reset counter when returning for a new cycle
                    if (!shooting && shotsFired >= 3) {
                        shotsFired = 0;
                    }

                    // Restore launcher RPM target before starting the next volley
                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
                        targetRPM = 2125;  // restore your launch RPM
                        startShooting3();
                    }

                    // Proceed after firing finishes
                    if (!shooting && shotsFired >= 3) {
                        returningToScore = false;
                        setPathState(4);
                    }
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
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
        follower.update();
        autonomousPathUpdate();
        updateShooting();
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
        actionTimer = new Timer(); // ← add this line
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        buildPaths();
        follower.setStartingPose(startPose);
        telemetry.addData("Shoot stage", shootStage);
        telemetry.addData("Shots fired", shotsFired);

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
}