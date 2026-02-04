package org.firstinspires.ftc.teamcode.auton; // make sure this aligns with class location
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Projects.newHWmap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Projects.newHWmap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue far")
public class iltFarBlueAuto extends OpMode {




    public newHWmap robot = new newHWmap();
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Limelight3A limelight;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean finishedShooting = false;


    private int pathState;




    public static double kP = 0.0125;
    public static double kI = 0.00015;
    public static double kD = 0.00000005;
    public static double kF = 0.0004208;
    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;
    final double TICKS_PER_REV_INTAKE = 146.44;


    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);


    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;
    PIDController turretpid = new PIDController(TP, TI, TD);


    final double TICKS_PER_REV_TURRET = 294.0;
    final double GEAR_RATIO_TURRET = 0.3953;
    final double MAX_DEGREES = 70;
    final double MIN_POWER_TO_MOVE = 0.05;


    private boolean shooting = false;
    private int shotsFired = 0;


    // Dynamic RPM Variable
    private double currentFlywheelTargetRPM = 1944;
    private static final double FLYWHEEL_ALLOWED_ERR_RPM = 100;


    private static final double FEED_TIME_SEC = 0.19;
    private static final double BETWEEN_SHOTS_MIN_SEC = 0.1;


    private boolean feeding = false;
    private double feedStartTime = 0.0;
    private double lastFeedEndTime = -999.0;
    private final int ticksPerRevLauncher = 28;


    public double getdistance(double ta){
        double scale = 10;
        double distance = scale/ta;
        return(distance);
    }


    public void startIntake() {
        robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 500 / 60);
        robot.shootServo.setPosition(0.5);
    }


    public void stopIntake() {
        robot.intake.setPower(0);
        robot.shootServo.setPosition(0.5);
    }


    public void startShooting3() {
        shooting = true;
        finishedShooting = false;
        shotsFired = 0;
        feeding = false;
        lastFeedEndTime = -999.0;
        actionTimer.resetTimer();


//        robot.lift.setTargetPosition(-55);
//        robot.lift.setPower(1);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void stopShooting() {
        robot.flywheel.setPower(0);
        robot.intake.setPower(0);
        robot.shootServo.setPosition(0.5);
        shooting = false;
        feeding = false;
        finishedShooting = true;


        robot.lift.setTargetPosition(1);
        robot.lift.setPower(-1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void updateShooting() {
        if (!shooting) {
            feeding = false;
            return;
        }


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double distance = getdistance(result.getTa());
            currentFlywheelTargetRPM = (415.2 * Math.log(distance)) + 1130;
        }


        double targetTicksPerSec = currentFlywheelTargetRPM / 60.0 * ticksPerRevLauncher;
        double measuredTicksPerSec = robot.flywheel.getVelocity();


        double ffOutput = feedforward.calculate(targetTicksPerSec);
        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);


        double launcherPower = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
        robot.flywheel.setPower(launcherPower);


        double measuredRPM = measuredTicksPerSec / ticksPerRevLauncher * 60.0;
        double flywheelErrRPM = Math.abs(measuredRPM - currentFlywheelTargetRPM);


        double now = actionTimer.getElapsedTimeSeconds();
        boolean flywheelReady = (flywheelErrRPM <= FLYWHEEL_ALLOWED_ERR_RPM);
        boolean cooldownDone = (now - lastFeedEndTime) >= BETWEEN_SHOTS_MIN_SEC;



        if (shotsFired >= 3) {
            stopShooting();
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
            robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1100 / 60);
            robot.shootServo.setPosition(0);
            if ((now - feedStartTime) >= FEED_TIME_SEC) {
                feeding = false;
                lastFeedEndTime = now;
                shotsFired++;
                robot.intake.setPower(0);
                robot.shootServo.setPosition(0.5);
                telemetry.addData("shotsFired",shotsFired);
            }
        }
    }


    public void updateTurret() {
        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            boolean tagFound = false;


            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == 20) {
                    tagFound = true;
                    double targetX = fr.getTargetXDegrees();
                    double turretpidOutput = turretpid.calculate(0, targetX);


                    double turretfeedforward = 0;
                    if (Math.abs(turretpidOutput) > 0.01) {
                        turretfeedforward = Math.signum(turretpidOutput) * MIN_POWER_TO_MOVE;
                    }


                    double motorPower = -(turretpidOutput + turretfeedforward);


                    int encoderTicks = robot.turret.getCurrentPosition();
                    double turretDegrees = (encoderTicks / (TICKS_PER_REV_TURRET / GEAR_RATIO_TURRET)) * 360.0;


                    if ((turretDegrees >= MAX_DEGREES && motorPower > 0) ||
                            (turretDegrees <= -MAX_DEGREES && motorPower < 0)) {
                        motorPower = 0;
                    }


                    robot.turret.setPower(motorPower);
                    break;
                }
            }
            if (!tagFound) {
                robot.turret.setPower(0);
            }
        } else {
            robot.turret.setPower(0);
        }
    }






    private final Pose startPose = new Pose(59, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(51, 11, Math.toRadians(107));//107 // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pick1 = new Pose(40,34, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(13, 34, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pick2 = new Pose(8.5, 27, Math.toRadians(210));
    private final Pose pickup2Pose = new Pose(8, 10, Math.toRadians(210));
    private final Pose pickup3Pose = new Pose(17, 8, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose backshot = new Pose(9, 8, Math.toRadians(180));


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, grabPickup22, scorePickup2, grabPickup3, scorePickup3;
    private PathChain backshots;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        backshots = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,backshot))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(.6)
                .addPath(new BezierLine(backshot,pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(.6)
                .addPath(new BezierLine(pickup3Pose,backshot))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(.6)
                .addPath(new BezierLine(backshot,pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(.6)

                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pick1))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pick1.getHeading())
                .addPath(new BezierLine(pick1, pickup1Pose))
                .setLinearHeadingInterpolation(pick1.getHeading(), pickup1Pose.getHeading())
                .setTValueConstraint(1)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setTValueConstraint(1)

                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath((new BezierLine(scorePose, pick2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pick2.getHeading())
                .setTValueConstraint(1)
                .build();
        grabPickup22 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(210))
                .setTValueConstraint(1)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .setTValueConstraint(1)
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))

                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup3Pose.getHeading())
                .setTValueConstraint(.9)
                .build();


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(follower.getPose(),new Pose(40,30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .setTValueConstraint(1)
                .build();
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
                if (!follower.isBusy()) {
                    /* Score Preload */
                    startShooting3();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (finishedShooting) {

                    /* Grab Sample */
                    startIntake();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(grabPickup1, .8,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    stopIntake();



                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(scorePickup1,.8, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    startShooting3();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setPathState(5);
                }
                break;

            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (finishedShooting) {

                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(grabPickup2,.8, true);
                    setPathState(6);
                }
                break;
            case 6:
            if (!follower.isBusy()) {

                /* Grab Sample */
                startIntake();
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                follower.followPath(grabPickup22,.6, true);
                startIntake();
                setPathState(7);
            }
            break;

            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    stopIntake();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2,.8, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    /* Score Preload */
                    startShooting3();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setPathState(9);
                }
                break;

            case 9:
                if (finishedShooting) {
                    /* Grab Sample */
                    startIntake();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup3,.6, true);

                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(backshots,.8, false);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the roboet's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    stopIntake();
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    follower.followPath(scorePickup3,.8, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    startShooting3();
                    setPathState(13);
                }
                break;


            case 13:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        updateTurret();


        if (shooting) {
            updateShooting();
        }


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
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


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


        buildPaths();
        follower.setStartingPose(startPose);
    }


    @Override public void init_loop() {}
    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); }
    @Override public void stop() {
        limelight.stop();
    }
}
