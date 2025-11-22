package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarBlueAuto", group = "fruitauto")
public class FarBlueAuto extends OpMode {

    public HWMap robot = new HWMap();
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean returningToScore = false;
    private boolean slowingForIntake = false;
    private double oldMaxPower = 1.0;

    private int pathState;
    private int nextState = -1;

    private final Pose startPose  = new Pose(62.72691807542263,  8.801040312093622, Math.toRadians(90));
    private final Pose scorePose  = new Pose(58.23276983094928, 18.725617685305593, Math.toRadians(114)); //116
    private final Pose pickup1Pose      = new Pose(41.941482444733424, 35.39141742522757, Math.toRadians(180));
    private final Pose pickup1intakePose= new Pose(10.040312093628103, 35.39141742522757, Math.toRadians(180));
    private final Pose parkPose      = new Pose(35.94950911640954, 8.280504908835908, Math.toRadians(180));

//    private final Pose pickup2Pose      = new Pose(41.941482444733424, 59.17295188556567, Math.toRadians(180));
    private final Pose pickup2intakePose= new Pose(10.040312093628103, 59.17295188556567, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(41.94148244473342, 35.39141742522757, Math.toRadians(180));
    private final Pose pickup3intakePose = new Pose(10.04031209362809, 35.39141742522757, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, intakePickup1, scorePickup1, parkPath, grabPickup2, intakePickup2, scorePickup2;

    public static double kP = 0.001;
    public static double kI = 0.0006;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private boolean shooting = false;
    private int shootStage = 0;
    private int shotsFired = 0;
    private double targetRPM = 3000; //2100 close
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
                if (errorRPM < 75 || actionTimer.getElapsedTimeSeconds() > 4.0) {
                    shootStage = 1;
                    actionTimer.resetTimer();
                }
                break;

            case 1:
                double measuredRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
                double allowedError = 60;

                robot.launcher.setVelocity(targetRPM / 60.0 * ticksPerRevLauncher);

                if (measuredRPM >= targetRPM - allowedError) {
                    if (actionTimer.getElapsedTimeSeconds() >= 0.2) {//0.07
                        robot.intake.setPower(0.75);
                        robot.intakeServo.setPower(1);
                    }
                    if (actionTimer.getElapsedTimeSeconds() >= 0.35) {//0.17 0.22
                        robot.intake.setPower(0);
                        robot.intakeServo.setPower(1);
                        shotsFired++;
                        actionTimer.resetTimer();

                        if (shotsFired >= 3) {
                            shootStage = 2;
                        }
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 2:
                if (errorRPM < 75 && actionTimer.getElapsedTimeSeconds() > 0.40) {//50, 0.3
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
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
                shooting = false;
                break;
        }
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setTranslationalConstraint(0.02);
        scorePreload.setHeadingConstraint(0.02);

//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();

//        intakePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, pickup1intakePose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1intakePose.getHeading())
//                .build();
//
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1intakePose, scorePose))
//                .setLinearHeadingInterpolation(pickup1intakePose.getHeading(), scorePose.getHeading())
//                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();

//        intakePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, pickup2intakePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2intakePose.getHeading())
//                .build();

//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2intakePose, scorePose))
//                .setLinearHeadingInterpolation(pickup2intakePose.getHeading(), scorePose.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            // PRELOAD SCORE
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            // SHOOT + GO TO PICKUP 1
            case 1:
                if (!follower.isBusy()) {
                    double headingError = Math.abs(follower.getPose().getHeading() - scorePose.getHeading());

                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
                        startShooting3();
                    }

                    if (!shooting && shotsFired >= 3) {
                        shotsFired = 0;
                        targetRPM = 0;
                        follower.followPath(parkPath, 0.5, true); //0.7
                        setPathState(2);
                    }
                }
                break;

            // PICKUP 1
//            case 2:
//
//                if (!slowingForIntake) {
//                    if (!follower.isBusy()) {
//
//                        double currentRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
//
//                        if (currentRPM <= 100) { //100
//                            robot.intake.setPower(0.23); //0.65
//                            robot.intakeServo.setPower(1);
//
//                            follower.followPath(intakePickup1, 0.5, true);
//                            actionTimer.resetTimer();
//                            nextState = 3;
//
//                            slowingForIntake = true;   // <<< FIX
//                            actionTimer.resetTimer();  // Reset timer for case 21
//                        }
//                    } else {
//                        // Wait until intake path finishes
//                        if (!follower.isBusy()) {
//                            setPathState(21);
//                        }
//
//                    }
//                }
//                break;

            // UNIVERSAL INTAKE EXIT
//            case 21:
//                if (actionTimer.getElapsedTimeSeconds() < 0.6) {//.25
//                    robot.launcher.setPower(-0.55);
//                    robot.intake.setPower(0);
//                    robot.intakeServo.setPower(1);
//                }
//
//                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
//                    robot.launcher.setPower(0);
//                    robot.intake.setPower(0);
//                    robot.intakeServo.setPower(1);
//                    slowingForIntake = false;
//                    setPathState(nextState);
//                }
//                break;
//            case 21:
//                if (actionTimer.getElapsedTimeSeconds() < 0.5) {
//                    robot.launcher.setPower(-0.55);
//                    robot.intake.setPower(0);
//                    robot.intakeServo.setPower(1);
//                } else {
//                    robot.launcher.setPower(0);
//                    robot.intake.setPower(0);
//                    robot.intakeServo.setPower(1);
//
//                    slowingForIntake = false;
//                    setPathState(nextState);
//                }
//                if (actionTimer.getElapsedTimeSeconds() < 0.25) { //0.25
//                    robot.launcher.setPower(-0.55); //-0.2
//                } else {
//                    robot.launcher.setPower(0);
//                    robot.intake.setPower(0);
//                    robot.intakeServo.setPower(1); //0
//
//                    slowingForIntake = false;   // <<< FIX
//
//                    setPathState(nextState);
//                }
//                break;

                // RETURN + SHOOT PICKUP 1
//            case 3:
//                if (!follower.isBusy()) {
//                    double headingError = Math.abs(follower.getPose().getHeading() - scorePose.getHeading());
//
//                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
//                        targetRPM = 3000; //2125
//                        startShooting3();
//                    }
//
//                    if (!shooting && shotsFired >= 3) {
//                        shotsFired = 0;
//                        targetRPM = 0;
//                        follower.followPath(parkPath, 0.5, true); //0.7
//                        setPathState(4);
//                    }
//                }
//                break;

            case 2:
                if (!follower.isBusy() && !returningToScore) {
                    follower.followPath(parkPath, 0.7, true);
                    returningToScore = true;
                }

                if (!follower.isBusy() && returningToScore) {
                    returningToScore = false;
                    setPathState(-1);
                }
                break;

            // PICKUP 2
//            case 4:
//
//                if (!slowingForIntake) {
//                    if (!follower.isBusy()) {
//
//                        double currentRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
//
//                        if (currentRPM <= 100) {
//                            robot.intake.setPower(0.23);
//                            robot.intakeServo.setPower(1);
//
//                            follower.followPath(intakePickup2, 0.5, true);
//                            actionTimer.resetTimer();
//                            nextState = 5;
//
//                            slowingForIntake = true;   // <<< FIX
//                        }
//                    }
//                } else {
//                    if (!follower.isBusy()) {
//                        actionTimer.resetTimer();
//                        setPathState(21);
//                    }
//                }
//                break;
//
//            // SCORE PICKUP 2
//            case 5:
//                if (!returningToScore) {
//                    follower.followPath(scorePickup2, 0.7, true);
//                    returningToScore = true;
//                }
//
//                if (!follower.isBusy()) {
//
//                    double headingError = Math.abs(follower.getPose().getHeading() - scorePose.getHeading());
//
//                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
//                        targetRPM = 3040; //2125 close
//                        startShooting3();
//                    }
//
//                    if (!shooting && shotsFired >= 3) {
//                        returningToScore = false;
//                        shotsFired = 0;
//                        setPathState(-1);
//                    }
//                }
//                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        updateShooting();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shotsFired", shotsFired);
        telemetry.addData("shootStage", shootStage);
        telemetry.addData("shooting", shooting);

        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
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
    }

    @Override public void init_loop() {}
    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); }
    @Override public void stop() {}
}