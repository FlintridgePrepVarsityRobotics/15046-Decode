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

@Autonomous(name = "StrawberryAuto", group = "fruitauto")
public class StrawberryAuto extends OpMode {

    public HWMap robot = new HWMap();
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean returningToScore = false;

    private int pathState;

    // Red-side mirrored poses
    private final Pose startPose = new Pose(144 - 16.06530612244898, 118.92244897959183, Math.toRadians(180 - 144));
    private final Pose scorePose = new Pose(144 - 38.4, 96.39183673469388, Math.toRadians(180 - 135));
    private final Pose pickup1Pose = new Pose(144 - 44.27755102040816, 79, Math.toRadians(180 - 180));
    private final Pose pickupintakePose = new Pose(144 - 28.367, 84.24489795918367, Math.toRadians(180 - 180));

    private Path scorePreload;
    private PathChain grabPickup1, intakePickup1, scorePickup1;
    public static double kP = 0.0003;
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
                if (errorRPM < 75 || actionTimer.getElapsedTimeSeconds() > 4.0) {
                    shootStage = 1;
                    actionTimer.resetTimer();
                }
                break;

            case 1:
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
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setTranslationalConstraint(0.001);
        scorePreload.setHeadingConstraint(0.001);

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickupintakePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickupintakePose.getHeading())
                .setTranslationalConstraint(0.3)
                .setHeadingConstraint(0.5)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupintakePose, scorePose))
                .setLinearHeadingInterpolation(pickupintakePose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    double currentHeading = follower.getPose().getHeading();
                    double headingError = Math.abs(currentHeading - scorePose.getHeading());

                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
                        startShooting3();
                    }

                    if (!shooting && shotsFired >= 3) {
                        targetRPM = 0;
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    double currentRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
                    if (currentRPM <= 100) {
                        robot.intake.setPower(0.65);
                        robot.intakeServo.setPower(1);
                        follower.followPath(intakePickup1, true);
                        actionTimer.resetTimer();
                        setPathState(21);
                    }
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 0.25) {
                        robot.launcher.setPower(-0.2);
                        robot.intake.setPower(-0.1);
                    } else {
                        robot.launcher.setPower(0);
                        robot.intake.setPower(0);
                        robot.intakeServo.setPower(0);
                        setPathState(3);
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

                    if (!shooting && shotsFired >= 3) shotsFired = 0;
                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
                        targetRPM = 2125;
                        startShooting3();
                    }
                    if (!shooting && shotsFired >= 3) {
                        returningToScore = false;
                        setPathState(-1);
                    }
                }
                break;
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
