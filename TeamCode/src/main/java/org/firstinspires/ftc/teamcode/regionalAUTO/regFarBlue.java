package org.firstinspires.ftc.teamcode.regionalAUTO;

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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Projects.newHWmap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "regFarBlue")
public class regFarBlue extends OpMode {

    public newHWmap robot = new newHWmap();

    private TelemetryManager panelsTelemetry;

    private Follower follower;
    private Limelight3A limelight;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private boolean finishedShooting = false;

    private int pathState;
    public static double IP = 0.0005;
    public static double II = 0;
    public static double ID = 0;
    public static double IF = 0.000048;
    public static double IS = 0.0;
    public static double IV = 0.0;
    public static double IA = 0.2;
    PIDFController Intakepidf = new PIDFController(IP, II, ID, IF);
    SimpleMotorFeedforward feedforwardIntake = new SimpleMotorFeedforward(IS, IV, IA);

    public static double TP = 0.02;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;

    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.00025;
    public static double kF = 0.00042;

    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    double setpointRPMIntake = 0;

    final double TICKS_PER_REV_INTAKE = 146.44;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    PIDController turretpid = new PIDController(TP, TI, TD);

    final double TICKS_PER_REV_TURRET = 294.0;
    final double GEAR_RATIO_TURRET = 0.3953;

    final double MAX_DEGREES = 70;
    double ticksPerRevIntake = 101.08;

    final double MIN_POWER_TO_MOVE = 0.05;

    private boolean shooting = false;
    private int shotsFired = 0;

    private double currentFlywheelTargetRPM = 1944;

    private static final double FLYWHEEL_ALLOWED_ERR_RPM = 20;

    private static final double FEED_TIME_SEC = .5;
    private static final double BETWEEN_SHOTS_MIN_SEC = 0.5;

    private boolean feeding = false;
    double dergs = 0;

    private double feedStartTime = 0.0;
    private double lastFeedEndTime = -999.0;

    private final int ticksPerRevLauncher = 28;
    double turretMotorPower = 0;
    private boolean turretManualControl = false;

    private final Pose startPose = new Pose(62.000, 7.625, Math.toRadians(90));

    public PathChain Spike3, Spike3Intake, Scoring, Corner, CornerIntake, Scoring2;
    public PathChain BackshotCurve1, BackshotCurve2, Backshot3, Scoring3, Park;

    public double getdistance(double ta){
        double scale = 10;
        double distance = scale/ta;
        return(distance);
    }

    public void startIntake(){
        double targetTicksPerSecIntake = setpointRPMIntake/60 * ticksPerRevIntake;
        double measuredTicksPerSecIntake = robot.intake.getVelocity();
        double ffOutputIntake = feedforwardIntake.calculate(targetTicksPerSecIntake);
        double pidOutputIntake = Intakepidf.calculate(measuredTicksPerSecIntake,targetTicksPerSecIntake);
        double combinedOutputIntake = ffOutputIntake + pidOutputIntake;
        combinedOutputIntake = Math.max(-1.0, Math.min(1.0, combinedOutputIntake));
        robot.intake.setPower(combinedOutputIntake);
    }

    public void stopIntake() {
        setpointRPMIntake = 0;
        robot.intake.setPower(0);
        robot.shootServo.setPosition(0.5);
    }

    public void startShooting3(double targetRPM) {
        shooting = true;
        finishedShooting = false;
        shotsFired = 0;
        feeding = false;
        lastFeedEndTime = -999.0;
        currentFlywheelTargetRPM = targetRPM;
        actionTimer.resetTimer();
    }
    public void holdTurret() {
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(1);
    }

    public void stopShooting() {
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
            double targetTicksPerSec = currentFlywheelTargetRPM / 60.0 * ticksPerRevLauncher;
            double measuredTicksPerSec = robot.flywheel.getVelocity();
            double ffOutput = feedforward.calculate(targetTicksPerSec);
            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);
            double launcherPower = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
            robot.flywheel.setPower(launcherPower);
            feeding = false;
            return;
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

        if (shotsFired >= 4) {
            stopShooting();
            robot.shootServo.setPosition(0.5);
            return;
        }

        if (!feeding) {
            setpointRPMIntake = 0;
            startIntake();
            robot.shootServo.setPosition(0.5);
            robot.intake.setPower(0);

            if (flywheelReady && cooldownDone) {
                feeding = true;
                feedStartTime = now;
            }

        } else {
            setpointRPMIntake = 1000;
            startIntake();
            robot.shootServo.setPosition(0.0);

            if ((now - feedStartTime) >= FEED_TIME_SEC) {

                feeding = false;
                lastFeedEndTime = now;
                shotsFired++;

                robot.shootServo.setPosition(0.5);
                robot.intake.setPower(0);

            }
        }
    }
//    public void updateTurret(){
//        double currDegs = robot.turret.getCurrentPosition() /
//                (TICKS_PER_REV_TURRET / GEAR_RATIO_TURRET) * 360.0;
//        if (Math.abs(currDegs - dergs) > 2) {
//            turretMotorPower = turretpid.calculate(currDegs, dergs);
//        }else{
//            turretMotorPower = 0;
//        }
//        robot.turret.setPower(turretMotorPower);
//    }
public void updateTurret() {
    double currDegs = robot.turret.getCurrentPosition() /
            (TICKS_PER_REV_TURRET / GEAR_RATIO_TURRET) * 360.0;

    if (!turretManualControl) {
        if (Math.abs(currDegs - dergs) <= 3) {
            turretManualControl = true;
            robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("pidturretcontroll on");
        }
        return;
    }

    // PID holding
    if (Math.abs(currDegs - dergs) > 3) {
        turretMotorPower = turretpid.calculate(currDegs, dergs);
    } else {
        turretMotorPower = 0;
    }
    robot.turret.setPower(turretMotorPower);
}
//    public void setTurretAngle(double degrees) {
//
//        int targetTicks = (int) Math.round(degrees * (TICKS_PER_REV_TURRET / GEAR_RATIO_TURRET) / 360.0);
//
//        robot.turret.setTargetPosition(targetTicks);
//        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.turret.setPower(0.6);
//
//    }
public void setTurretAngle(double degrees) {
    dergs = degrees;
    turretManualControl = false;

    int targetTicks = (int) Math.round(degrees *
            (TICKS_PER_REV_TURRET / GEAR_RATIO_TURRET) / 360.0);

    robot.turret.setTargetPosition(targetTicks);
    robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.turret.setPower(0.6);
}
    public void buildPaths() {

        Spike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(62.000, 7.625), new Pose(62.000, 24.000), new Pose(44.000, 35.200)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

        Spike3Intake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(44.000, 35.200), new Pose(14.000, 35.200)))
                .setTangentHeadingInterpolation().build();

        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(14.000, 35.200), new Pose(54.000, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111)).build();

        Corner = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(54.000, 15.000), new Pose(34.250, 30.000), new Pose(14, 25.000)))
                .setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(220)).build();

        CornerIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(14, 25.000), new Pose(14, 6)))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(220)).build();

        Scoring2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(14, 6), new Pose(38.250, 20.000), new Pose(54.000, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(111)).build();

        BackshotCurve1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(54.000, 15.000), new Pose(50.125, 15.000), new Pose(36.250, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(180)).build();

        BackshotCurve2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(36.250, 15.000), new Pose(25.375, 15.000), new Pose(12.500, 11.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        Backshot3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(12.500, 11.000), new Pose(34.000, 18.000), new Pose(12.500, 25.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        Scoring3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.500, 25.000), new Pose(54.000, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111)).build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(54.000, 15.000), new Pose(59.000, 29.000)))
                .setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(111)).build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setTurretAngle(24);
                startShooting3(2425); // test rpm
                setPathState(1);

                break;


            case 1:
                if (finishedShooting) {
                    setTurretAngle(4.5);
                    follower.followPath(Spike3, .8, true);
                    setPathState(2);
                }
                break;


            case 2:
                if(!follower.isBusy()) {
                    setpointRPMIntake = 800;
                    startIntake();
                    follower.followPath(Spike3Intake, .6, true);
                    setPathState(3);
                }
                break;


            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(Scoring, .9, true);
                    setPathState(4);
                }
                break;


            case 4:
                if(!follower.isBusy()) {
                    stopIntake();
                    setTurretAngle(4.5);
                    startShooting3(2380); // test rpm
                    setPathState(5);
                }
                break;


            case 5:
                if (finishedShooting) {
                    follower.followPath(Corner, .9, true);
                    setPathState(6);
                }
                break;


            case 6:
                if(!follower.isBusy()) {
                    setpointRPMIntake = 1000;
                    startIntake();
                    follower.followPath(CornerIntake, .5, true);
                    setPathState(7);
                }
                break;


            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(Scoring2, .6, true);
                    setPathState(8);
                }
                break;


            case 8:
                if(!follower.isBusy()) {
                    stopIntake();
                    setTurretAngle(4.5);
                    startShooting3(2380);
                    setPathState(13);
                }
                break;

//
//            case 9:
//                if (finishedShooting) {
//                    follower.followPath(BackshotCurve1, .6, true);
//                    setPathState(10);
//                }
//                break;
//
//
//            case 10:
//                if(!follower.isBusy()) {
//                    follower.followPath(BackshotCurve2, .6, true);
//                    setPathState(11);
//                }
//                break;
//
//
//            case 11:
//                if(!follower.isBusy()) {
//                    setpointRPMIntake = 1000;
//                    startIntake();
//                    follower.followPath(Backshot3, .6, true);
//                    setPathState(12);
//                }
//                break;
//
//
//            case 12:
//                if(!follower.isBusy()) {
//                    follower.followPath(Scoring3, .6, true);
//                    setPathState(13);
//                }
//                break;


            case 13:
                if(finishedShooting) {
                    stopIntake();
                    follower.followPath(Park, .6, true);
                    setPathState(14);
                }
                break;


            case 14:
                if(!follower.isBusy()) {
                    setPathState(-1);
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
        updateTurret();
//        telemetry.addLine("updatingTurret");
//        telemetry.addData("turretmotor power",turretMotorPower);
//        telemetry.addData("flywheel measured RPM", robot.flywheel.getVelocity() / ticksPerRevLauncher * 60.0);
//        telemetry.addData("flywheel target RPM", currentFlywheelTargetRPM);
//        telemetry.addData("shots fired", shotsFired);
//        telemetry.addData("feeding", feeding);
//        telemetry.addData("flywheelReady", Math.abs(robot.flywheel.getVelocity() / ticksPerRevLauncher * 60.0 - currentFlywheelTargetRPM) <= FLYWHEEL_ALLOWED_ERR_RPM);
telemetry.addData("shotsfired",shotsFired);

        updateShooting();



        telemetry.addData("target rpm", currentFlywheelTargetRPM);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
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


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        panelsTelemetry.update(telemetry);


        buildPaths();
        follower.setStartingPose(startPose);
    }


    @Override public void init_loop() {}


    @Override public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override public void stop() {
        limelight.stop();
    }
}



