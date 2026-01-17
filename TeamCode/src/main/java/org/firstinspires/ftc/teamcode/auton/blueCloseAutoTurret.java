package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Projects.newHWmap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueCloseAuto", group = "fruitauto")
public class blueCloseAutoTurret extends OpMode {

    public newHWmap robot = new newHWmap();
    private TelemetryManager panelsTelemetry;
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean finishedShooting = false;
    //Eversons very good stuff
    private boolean intakeServoRunning = false;
    private double intakeServoStartTime = 0.0;

    private static final double INTAKE_SERVO_RUN_TIME = 1.0;

    //Eversons very good stuff
    private boolean returningToScore = false;
    private boolean slowingForIntake = false;

    private int pathState;
    private int nextState = -1;

    private final Pose startPose          = new Pose(18.159947984395316, 122.46553966189856, Math.toRadians(143));
    private final Pose scorePose          = new Pose(36.88556566970091,  102.24187256176852, Math.toRadians(135));
    private final Pose pickup1Pose        = new Pose(45.8777633289987, 83.32899869960988, Math.toRadians(180));
    private final Pose pickup1intakePose  = new Pose(15.5, 83.32899869960980, Math.toRadians(180));
    private final Pose pickup2Pose        = new Pose(45.8777633289987, 59.17295188556567, Math.toRadians(180));
    private final Pose pickup2intakePose  = new Pose(15.534460338101432, 59.17295188556567, Math.toRadians(180));
    private final Pose park  = new Pose(12.848979591836734, 102.66122448979593, Math.toRadians(90));
    private final Pose pickup3Pose        = new Pose(45.8777633289987, 35.39141742522757, Math.toRadians(180));
    private final Pose pickup3intakePose  = new Pose(12.534460338101432, 35.39141742522757, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, intakePickup1, scorePickup1, grabPickup2, intakePickup2, scorePickup2, parking;

    public static double kP = 0.0125;

    public static double kI = 0.00015;
    public static double kD = 0.00000005;
    public static double kF = 0.0004208;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;
    final double TICKS_PER_REV_INTAKE = 146.44;



    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    PIDController turretpid = new PIDController(TP, TI, TD);
    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;
    private boolean shooting = false;
    private int shotsFired = 0;

    private static final double FLYWHEEL_TARGET_RPM = 2200;
    private static final double FLYWHEEL_ALLOWED_ERR_RPM = 100;
    private static final double INTAKE_TARGET_RPM = 500;

    private static final double FEED_TIME_SEC = 0.2;
    private static final double BETWEEN_SHOTS_MIN_SEC = 0;

    private boolean feeding = false;
    private double feedStartTime = 0.0;
    private double lastFeedEndTime = -999.0;

    private final int ticksPerRevLauncher = 28;
    private final int ticksPerRevIntake = 224;
    //Eversons very good stuff
    public void runIntakeServoOneSecond() {
        intakeServoRunning = true;
        intakeServoStartTime = opmodeTimer.getElapsedTimeSeconds();
    }
    //Everson very good stuff
    public void updateIntakeServo() {
        if (!intakeServoRunning) return;

        double now = opmodeTimer.getElapsedTimeSeconds();
        if (now - intakeServoStartTime >= INTAKE_SERVO_RUN_TIME) {
            intakeServoRunning = false;
        }
    }

    public void startShooting3() {
        shooting = true;
        finishedShooting = false;
        shotsFired = 0;
        feeding = false;
        lastFeedEndTime = -999.0;
        actionTimer.resetTimer();
    }


    public void updateShooting() {
        if (!shooting) {
            feeding = false;
            return;
        }


        double targetTicksPerSec = FLYWHEEL_TARGET_RPM / 60.0 * ticksPerRevLauncher;
        double measuredTicksPerSec = robot.flywheel.getVelocity();

        double ffOutput = feedforward.calculate(targetTicksPerSec);
        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

        double launcherPower = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
        robot.flywheel.setPower(launcherPower);

        double measuredRPM = measuredTicksPerSec / ticksPerRevLauncher * 60.0;
        double flywheelErrRPM = Math.abs(measuredRPM - FLYWHEEL_TARGET_RPM);

        double now = actionTimer.getElapsedTimeSeconds();
        boolean flywheelReady = (flywheelErrRPM <= FLYWHEEL_ALLOWED_ERR_RPM);
        boolean cooldownDone = (now - lastFeedEndTime) >= BETWEEN_SHOTS_MIN_SEC;

        if (shotsFired >= 3) {
            robot.flywheel.setPower(0);
            robot.intake.setPower(0);
            shooting = false;
            feeding = false;
            finishedShooting = true;
            return;
        }


        if (!flywheelReady) {
            robot.intake.setPower(0);
            feeding = false;
            return;
        }

        if (!feeding) {
            if (cooldownDone) {
                feeding = true;
                feedStartTime = now;
            } else {
                robot.intake.setPower(0);
            }
        }

        if (feeding) {
            robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 500 / 60);
//            robot.intake.setVelocity(TICKS_PER_REV_INTAKE*1100/60); //test code
            robot.shootServo.setPosition(0.5);
            if ((now - feedStartTime) >= FEED_TIME_SEC) {
                feeding = false;
                lastFeedEndTime = now;
                shotsFired++;
                robot.intake.setPower(0);
            }
        }
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setTranslationalConstraint(0.02);
        scorePreload.setHeadingConstraint(0.02);

        parking = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    double headingError = Math.abs(follower.getPose().getHeading() - scorePose.getHeading());
                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
                        startShooting3();
                    }
                    if (!shooting && shotsFired >= 3) {
                        shotsFired = 0;
                        finishedShooting = false;
                        follower.followPath(grabPickup1, 0.8, true);
                        setPathState(2);
                    }
                }
                break;



            case 2:
                follower.followPath(parking, .5, true);
                setPathState(-1);
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
        if (shooting) {
            updateShooting();
        }
        updateIntakeServo();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shotsFired", shotsFired);
        telemetry.addData("shooting", shooting);
        telemetry.addData("feeding", feeding);
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

