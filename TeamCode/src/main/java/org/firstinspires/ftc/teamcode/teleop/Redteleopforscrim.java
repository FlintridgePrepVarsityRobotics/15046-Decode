package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import android.graphics.Color;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Config
@TeleOp(name = "redteleforscrim")
public class Redteleopforscrim extends LinearOpMode {

    public HWMap robot = new HWMap();
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();
    boolean reversingLauncher = false;
    boolean reversedBefore = false;

    // PIDF + Feedforward constants (starting values — tune these)
    // These gains are chosen so PIDF+FF outputs a motor power in [-1,1].
    public static double kP = 0.001;
    public static double kI = 0.0006;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;
    //INTAKE
    public static double IkP = 0.001;
    public static double IkI = 0.0006;
    public static double IkD = 0.0;
    public static double IkF = 0.0;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double IkS = 0.0;
    public static double IkV = 0.00042;
    public static double IkA = 0.0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    PIDFController pidfIntake = new PIDFController(IkP, IkI, IkD, IkF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    SimpleMotorFeedforward feedforwardIntake = new SimpleMotorFeedforward(IkS, IkV, IkA);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // --- Vision setup ---
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        int frameWidth = 640;
        int centerX = frameWidth / 2;
        int tolerance = 50; // pixels within which the tag is centered

        double speed = 1;
        boolean lastUp = false;
        boolean lastMid = false;
        boolean lastDown = false;
        boolean lastX = false;
        boolean bWasPressed = false;
        boolean isIntakeRunning = false;

        boolean intakeFull = false;
        double tagDist = 0;

        boolean color1 = false;
        boolean color2 = false;

        // For A-button toggle
        boolean lastAState = false;

        int ticksPerRev = 28;
        double setpointRPM = 0;
        double targetRPM = 0;

        // Ensure launcher has encoder mode set if you want velocity feedback
        robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColorSensor sensor1;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sensor1 = hardwareMap.get(ColorSensor.class, "sensor1");

        // get a reference to the distance sensor that shares the same name

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsv1[] = {0F, 0F, 0F};
        float hsv2[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        waitForStart();

        while (opModeIsActive()) {
            // Creating obj for PID Tuning
            TelemetryPacket packet = new TelemetryPacket();
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

            TelemetryPacket packetIntake = new TelemetryPacket();
            pidfIntake.setPIDF(IkP, IkI, IkD, IkF);
            feedforward = new SimpleMotorFeedforward(IkS, IkV, IkA);

            // color scale factor init
            Color.RGBToHSV(
                    (int) (robot.sensor1.red() * SCALE_FACTOR),
                    (int) (robot.sensor1.green() * SCALE_FACTOR),
                    (int) (robot.sensor1.blue() * SCALE_FACTOR),
                    hsv1
            );

            Color.RGBToHSV(
                    (int) (robot.sensor2.red() * SCALE_FACTOR),
                    (int) (robot.sensor2.green() * SCALE_FACTOR),
                    (int) (robot.sensor2.blue() * SCALE_FACTOR),
                    hsv2
            );
            telemetry.addData("Red", robot.sensor1.red());
            telemetry.addData("Green", robot.sensor1.green());
            telemetry.addData("Blue", robot.sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);
            float hue1 = hsv1[0];
            float hue2 = hsv2[0];

            boolean tagCentered = false;

            if(hue1 < 30){
                telemetry.addData("Color", "Red");
                color1 = false;
            }

            else if (hue1 < 60) {
                telemetry.addData("Color", "Orange");
                color1 = false;
            }

            else if (hue1 < 140){
                telemetry.addData("Color", "Yellow");
                color1 = false;

            }

            else if (hue1 < 250){ //green --> 160
                telemetry.addData("Color", "Green");
                color1 = true;
            }

            else if (hue1 < 260){
                telemetry.addData("Color", "Blue");
                color1 = false;

            }

            else if (hue1 < 270){ //purple --> 230-250
                telemetry.addData("Color", "Purple");
                color1 = true;
            }

            else{
                telemetry.addData("Color", "Red");
                color1 = false;
            }

            if(hue2 < 30){
                telemetry.addData("Color2", "Red");
                color2 = false;
            }

            else if (hue2 < 60) {
                telemetry.addData("Color2", "Orange");
                color2 = false;
            }

            else if (hue2 < 140){
                telemetry.addData("Color2", "Yellow");
                color2 = false;
            }

            else if (hue2 < 180){ //green --> 160
                telemetry.addData("Color2", "Green");
                color2 = true;
            }

            else if (hue2 < 200){
                telemetry.addData("Color2", "Blue");
                color2 = false;
            }

            else if (hue2 < 250){ //purple --> 230-250
                telemetry.addData("Color2", "Purple");
                color2 = true;
            }

            else{
                telemetry.addData("Color2", "Red");
                color2 = false;
            }

            // --- Driver control ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);

            // --- Launcher RPM Control ---
            boolean highSpeed = gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;

            // Update setpoint only when a D-pad button is newly pressed (rising edge),
            // so you don't keep re-setting it each loop.
            if (highSpeed && !lastUp) setpointRPM = 3200;
            if (midSpeed && !lastMid) setpointRPM = 2600;
            if (lowSpeed && !lastDown) setpointRPM = 2400;
            if (gamepad1.x && !lastX) setpointRPM = 0;

            // Measurements in ticks/sec
            double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
            double measuredTicksPerSec = robot.launcher.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;

            double ItargetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
            double ImeasuredTicksPerSec = robot.intake.getVelocity();
            double ImeasuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;
            // Feedforward baseline (returns value in same "command" units as gains —
            // we've chosen gains so this approximates motor power)
            double ffOutput = feedforward.calculate(targetTicksPerSec);

            // PIDF returns correction. Give it the measurement and target (also ticks/sec).
            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            double IntakepidOutput = pidfIntake.calculate(measuredTicksPerSec, targetTicksPerSec);
            // Combine and clamp to motor power range [-1, 1]
            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            double IcombinedOutput = ffOutput + IntakepidOutput;
            IcombinedOutput = Math.max(-1.0, Math.min(1.0, IcombinedOutput));

            // If the driver pressed D-pad (we want launcher behavior), apply combined power.
            // If the player pressed 'x' or dpad_down, those override below.
            robot.launcher.setPower(combinedOutput);

            if (color1 && color2){
                if (colorTimer.milliseconds() > 500 && !intakeFull){
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(1);
                    intakeFull = true;
                    reversingLauncher = true;
                    reversedBefore = false;
                    reverseTimer.reset();
                }
            }
            else{
                colorTimer.reset();
                intakeFull = false;
            }

            if (reversingLauncher && !reversedBefore) {
                robot.launcher.setPower(-0.5);
                robot.intakeServo.setPower(1);
                if (reverseTimer.milliseconds() >= 500) {
                    reversingLauncher = false;
                    robot.intakeServo.setPower(1);
                    reversedBefore = true;
                }
            }

            // --- Dpad down: reverse intake & launcher negative (manual) ---
            if (gamepad1.dpad_down) {
                robot.intake.setPower(-0.3);
                setpointRPM = -1000;
                double targetTicksPerSecDown = targetRPM / 60.0 * ticksPerRev;
                double downPower = feedforward.calculate(targetTicksPerSecDown);
                downPower = Math.max(-1.0, Math.min(1.0, downPower));
                robot.launcher.setPower(downPower);
            }
            // --- A Button -- toggle intake and shuts off when full
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState && !intakeFull) {
                // just pressed
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    robot.intake.setPower(0.25);
                    robot.intakeServo.setPower(1);
                    buttonTimer.reset();
                } else {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(1);
                }
            }
            lastAState = aNow;

            // --- B button: timed intake pulse ---
            if (gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 150) {
                if (!bWasPressed) {
                    buttonTimer.reset();
                    robot.intake.setPower(0.75);
                    robot.intakeServo.setPower(1);
                    bWasPressed = true;
                }
                if (buttonTimer.milliseconds() >= 170) {
                    robot.intake.setPower(0);
                    robot.intakeServo.setPower(1);
                }
            } else if (!isIntakeRunning) {
                bWasPressed = false;
                robot.intake.setPower(0);
                robot.intakeServo.setPower(1);
            }

            // --- Telemetry for tuning ---
            telemetry.addData("Setpoint RPM", setpointRPM);
            packet.put("Setpoint RPM", setpointRPM);
            telemetry.addData("Measured RPM", "%.1f", measuredRPM);
            packet.put("Measured RPM", measuredRPM);
            telemetry.addData("FF Output", "%.4f", ffOutput);
            packet.put("FF Output", ffOutput);
            telemetry.addData("PID Output", "%.4f", pidOutput);
            packet.put("PID Output", pidOutput);
            telemetry.addData("Combined (power)", "%.4f", combinedOutput);
            packet.put("Combined (power)", combinedOutput);

            telemetry.addData("Clear", sensor1.alpha());
            telemetry.addData("Red  ", sensor1.red());
            telemetry.addData("Green", sensor1.green());
            telemetry.addData("Blue ", sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);

            // --- AprilTag Centering (Y button) ---
            if (gamepad1.y) {
                if (!tagProcessor.getDetections().isEmpty()) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    if (tag.id == 24) {
                        double tagX = tag.center.x;
                        tagDist = tag.ftcPose.range;
                        telemetry.addData("Distance from Tag", "%.1f", tagDist);
                        centerX = frameWidth / 2;
                        if (tagDist > 100){centerX += 50;}
                        if (tagX < centerX - tolerance) {
                            robot.fRightWheel.setPower(0.2);
                            robot.bRightWheel.setPower(0.2);
                            robot.fLeftWheel.setPower(-0.2);
                            robot.bLeftWheel.setPower(-0.2);
                            telemetry.addLine("Turning left to center tag");
                        } else if (tagX > centerX + tolerance) {
                            robot.fRightWheel.setPower(-0.2);
                            robot.bRightWheel.setPower(-0.2);
                            robot.fLeftWheel.setPower(0.2);
                            robot.bLeftWheel.setPower(0.2);
                            telemetry.addLine("Turning right to center tag");
                        } else {
                            robot.fRightWheel.setPower(0);
                            robot.bRightWheel.setPower(0);
                            robot.fLeftWheel.setPower(0);
                            robot.bLeftWheel.setPower(0);
                            telemetry.addLine("Tag centered!");
                            tagCentered = true;
                        }
                        telemetry.addData("Tag X", tag.center.x);
                        telemetry.addData("Center", centerX);
                    } else {
                        telemetry.addLine("Tag detected but not ID 24");
                    }
                } else {
                    telemetry.addLine("No tags detected");
                }
            }
// While the robot is looking at the tag once, it will toggle intake off
            if (tagCentered && isIntakeRunning) {
                isIntakeRunning = false;
                robot.intake.setPower(0);
                robot.intakeServo.setPower(0);
            }
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            // store previous D-pad states
            lastUp = highSpeed;
            lastMid = midSpeed;
            lastDown = lowSpeed;
        }
    }
}



////package org.firstinspires.ftc.teamcode.auton;
////
////import com.arcrobotics.ftclib.controller.PIDFController;
////import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
////import com.pedropathing.follower.Follower;
////import com.pedropathing.geometry.BezierLine;
////import com.pedropathing.geometry.Pose;
////import com.pedropathing.paths.Path;
////import com.pedropathing.paths.PathChain;
////import com.pedropathing.util.Timer;
////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////import com.bylazar.telemetry.PanelsTelemetry;
////import com.bylazar.telemetry.TelemetryManager;
////
////import org.firstinspires.ftc.teamcode.Projects.HWMap;
////import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
////
////@Autonomous(name = "BlueberryAuto", group = "fruitauto")
////public class BlueberryAuto extends OpMode {
////
////    public HWMap robot = new HWMap();
////    private TelemetryManager panelsTelemetry;
////    private Follower follower;
////    private Timer pathTimer, actionTimer, opmodeTimer;
////    private boolean returningToScore = false;
////    private boolean slowingForIntake = false;
////    private double oldMaxPower = 1.0;
////
////    private int pathState;
////    private int nextState = -1;
////
////
////    private final Pose startPose          = new Pose(18.159947984395316, 122.46553966189856, Math.toRadians(143));
////    private final Pose scorePose          = new Pose(36.88556566970091,  102.24187256176852, Math.toRadians(135));
////    private final Pose pickup1Pose        = new Pose(45.8777633289987, 83.32899869960988, Math.toRadians(180));
////    private final Pose pickup1intakePose  = new Pose(23, 83.32899869960980, Math.toRadians(180)); //x: 21.534460338101432
////    private final Pose pickup2Pose        = new Pose(45.8777633289987, 59.17295188556567, Math.toRadians(180));
////    private final Pose pickup2intakePose  = new Pose(21.534460338101432, 59.17295188556567, Math.toRadians(180));
////    private final Pose pickup3Pose        = new Pose(45.8777633289987, 35.39141742522757, Math.toRadians(180));
////    private final Pose pickup3intakePose  = new Pose(21.534460338101432, 35.39141742522757, Math.toRadians(180));
////
////
////
////private Path scorePreload;
////    private PathChain grabPickup1, intakePickup1, scorePickup1, grabPickup2, intakePickup2, scorePickup2;
////    public static double kP = 0.001;
////    public static double kI = 0.0006;
////    public static double kD = 0.0;
////    public static double kF = 0.0;
////
////    public static double kS = 0.0;
////    public static double kV = 0.00042;
////    public static double kA = 0.0;
////    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
////    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
////    private boolean shooting = false;
////    private int shootStage = 0;
////    private int shotsFired = 0;
////    private double targetRPM = 2125;
////    private final int ticksPerRevLauncher = 28;
////
////    public void startShooting3() {
////        shooting = true;
////        shootStage = 0;
////        shotsFired = 0;
////        actionTimer.resetTimer();
////    }
////
////    public void updateShooting() {
////        if (!shooting) return;
////
////        double targetTicksPerSec = targetRPM / 60.0 * ticksPerRevLauncher;
////        double measuredTicksPerSec = robot.launcher.getVelocity();
////        double ffOutput = feedforward.calculate(targetTicksPerSec);
////        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);
////        double combinedOutput = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
////        robot.launcher.setPower(combinedOutput);
////
////        double errorRPM = Math.abs(measuredTicksPerSec / ticksPerRevLauncher * 60.0 - targetRPM);
////
////        switch (shootStage) {
////            case 0:
////                if (errorRPM < 75 || actionTimer.getElapsedTimeSeconds() > 4.0) {
////                    shootStage = 1;
////                    actionTimer.resetTimer();
////                }
////                break;
////
////            case 1:
////                robot.intake.setPower(0.67);
////                robot.intakeServo.setPower(1);
////                if (actionTimer.getElapsedTimeSeconds() > 0.15) {
////                    robot.intake.setPower(0);
////                    robot.intakeServo.setPower(0);
////                    shootStage = 2;
////                    actionTimer.resetTimer();
////                    shotsFired++;
////                }
////                break;
////
////            case 2:
////                if (errorRPM < 50 && actionTimer.getElapsedTimeSeconds() > 0.3) {
////                    if (shotsFired < 3) {
////                        shootStage = 1;
////                        actionTimer.resetTimer();
////                    } else {
////                        shootStage = 3;
////                    }
////                }
////                break;
////
////            case 3:
////                robot.launcher.setPower(0);
////                shooting = false;
////                break;
////        }
////    }
////
////    public void buildPaths() {
////        scorePreload = new Path(new BezierLine(startPose, scorePose));
////        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
////        scorePreload.setTranslationalConstraint(0.02);
////        scorePreload.setHeadingConstraint(0.02);
////
////        grabPickup1 = follower.pathBuilder()
////                .addPath(new BezierLine(scorePose, pickup1Pose))
////                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
////                .build();
////
////        intakePickup1 = follower.pathBuilder()
////                .addPath(new BezierLine(pickup1Pose, pickup1intakePose))
////                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1intakePose.getHeading())
////                .build();
////        scorePickup1 = follower.pathBuilder()
////                .addPath(new BezierLine(pickup1intakePose, scorePose))
////                .setLinearHeadingInterpolation(pickup1intakePose.getHeading(), scorePose.getHeading())
////                .build();
////
////        grabPickup2 = follower.pathBuilder()
////                .addPath(new BezierLine(scorePose, pickup2Pose))
////                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
////                .build();
////
////        intakePickup2 = follower.pathBuilder()
////                .addPath(new BezierLine(pickup2Pose, pickup2intakePose))
////                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2intakePose.getHeading())
////                .build();
////        scorePickup2 = follower.pathBuilder()
////                .addPath(new BezierLine(pickup2intakePose, scorePose))
////                .setLinearHeadingInterpolation(pickup2intakePose.getHeading(), scorePose.getHeading())
////                .build();
////    }
////
////
////    public void autonomousPathUpdate() {
////        switch (pathState) {
////            case 0:
////                follower.followPath(scorePreload,true);
////                setPathState(1);
////                break;
////
////            case 1:
////                if (!follower.isBusy()) {
////                    double currentHeading = follower.getPose().getHeading();
////                    double headingError = Math.abs(currentHeading - scorePose.getHeading());
////
////                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
////                        startShooting3();
////                    }
////
////                    if (!shooting && shotsFired >= 3) {
////                        targetRPM = 0;
////                        shotsFired = 0;
////                        follower.followPath(grabPickup1, 0.7,true);
////                        setPathState(2);
////                    }
////                }
////                break;
////
////            case 2:
////                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
////                    follower.breakFollowing();
////                    actionTimer.resetTimer();
////                    nextState = 3;
////                    setPathState(21);
////                    break;
////                }
////
////                if (!slowingForIntake) {
////                    if (!follower.isBusy()) {
////                        double currentRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
////                        if (currentRPM <= 100) {
////                            robot.intake.setPower(0.65);
////                            robot.intakeServo.setPower(1);
////                            follower.followPath(intakePickup1,0.5,true);
////                            actionTimer.resetTimer();
////                            nextState = 3;
////                        }
////                    }
////                } else if (!follower.isBusy()) {
////                    setPathState(21);
////                }
////                break;
////
////            case 21:
////                if (!follower.isBusy()) {
////                    if (actionTimer.getElapsedTimeSeconds() < 0.25) {
////                        robot.launcher.setPower(-0.2);
////                    } else {
////                        robot.launcher.setPower(0);
////                        robot.intake.setPower(0);
////                        robot.intakeServo.setPower(0);
////                        setPathState(nextState);
////                    }
////                }
////                break;
////
////            case 3:
////                if (!returningToScore) {
////                    follower.followPath(scorePickup1, 0.7,true);
////                    returningToScore = true;
////                }
////                if (!follower.isBusy()) {
////                    double currentHeading = follower.getPose().getHeading();
////                    double headingError = Math.abs(currentHeading - scorePose.getHeading());
////                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
////                        targetRPM = 2125;
////                        startShooting3();
////                    }
////                    if (!shooting && shotsFired >= 3) {
////                        returningToScore = false;
////                        shotsFired = 0;
////                        follower.followPath(grabPickup2,0.7,true);
////                        setPathState(4);
////                    }
////                }
////                break;
////
////            case 4:
////                if (!slowingForIntake) {
////                    if (!follower.isBusy()) {
////                        double currentRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
////                        if (currentRPM <= 100) {
////                            robot.intake.setPower(0.65);
////                            robot.intakeServo.setPower(1);
////                            follower.followPath(intakePickup2, 0.5, true);
////                            actionTimer.resetTimer();
////                            nextState = 5;
////                        }
////                    }
////                } else {
////                    if (!follower.isBusy()) {
////                        setPathState(21);
////                    }
////                }
////                break;
////
////            case 5:
////                if (!returningToScore) {
////                    follower.followPath(scorePickup2, 0.7,true);
////                    returningToScore = true;
////                }
////                if (!follower.isBusy()) {
////                    double currentHeading = follower.getPose().getHeading();
////                    double headingError = Math.abs(currentHeading - scorePose.getHeading());
////
////                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
////                        targetRPM = 2125;
////                        startShooting3();
////                    }
////                    if (!shooting && shotsFired >= 3) {
////                        returningToScore = false;
////                        shotsFired = 0;
////                        setPathState(-1);
////                    }
////                }
////                break;
////        }
////    }
////
////    public void setPathState(int pState) {
////        pathState = pState;
////        pathTimer.resetTimer();
////    }
////
////    @Override
////    public void loop() {
////        follower.update();
////        autonomousPathUpdate();
////        updateShooting();
////
////        telemetry.addData("path state", pathState);
////        telemetry.addData("x", follower.getPose().getX());
////        telemetry.addData("y", follower.getPose().getY());
////        telemetry.addData("heading", follower.getPose().getHeading());
////        telemetry.addData("shotsFired", shotsFired);
////        telemetry.addData("shootStage", shootStage);
////        telemetry.addData("shooting", shooting);
////
////        telemetry.update();
////    }
////
////    @Override
////    public void init() {
////        pathTimer = new Timer();
////        actionTimer = new Timer();
////        opmodeTimer = new Timer();
////        opmodeTimer.resetTimer();
////        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
////
////        robot.init(hardwareMap);
////        follower = Constants.createFollower(hardwareMap);
////        follower.setStartingPose(startPose);
////
////        panelsTelemetry.debug("Status", "Initialized");
////        panelsTelemetry.update(telemetry);
////
////        buildPaths();
////        follower.setStartingPose(startPose);
////    }
////
////    @Override public void init_loop() {}
////    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); }
////    @Override public void stop() {}
////}
////
//
//package org.firstinspires.ftc.teamcode.auton;
//
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//
//import org.firstinspires.ftc.teamcode.Projects.HWMap;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "BlueCloseAuto", group = "fruitauto")
//public class BlueCloseAuto extends OpMode {
//
//    public HWMap robot = new HWMap();
//    private TelemetryManager panelsTelemetry;
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private boolean returningToScore = false;
//    private boolean slowingForIntake = false;
//    private double oldMaxPower = 1.0;
//
//    private int pathState;
//    private int nextState = -1;
//
//    private final Pose startPose          = new Pose(18.159947984395316, 122.46553966189856, Math.toRadians(143));
//    private final Pose scorePose          = new Pose(36.88556566970091,  102.24187256176852, Math.toRadians(135));
//    private final Pose pickup1Pose        = new Pose(45.8777633289987, 83.32899869960988, Math.toRadians(180));
//    private final Pose pickup1intakePose  = new Pose(20.5, 83.32899869960980, Math.toRadians(180));
//    private final Pose pickup2Pose        = new Pose(45.8777633289987, 59.17295188556567, Math.toRadians(180));
//    private final Pose pickup2intakePose  = new Pose(20.534460338101432, 59.17295188556567, Math.toRadians(180));
//    private final Pose pickup3Pose        = new Pose(45.8777633289987, 35.39141742522757, Math.toRadians(180));
//    private final Pose pickup3intakePose  = new Pose(21.534460338101432, 35.39141742522757, Math.toRadians(180));
//
//    private Path scorePreload;
//    private PathChain grabPickup1, intakePickup1, scorePickup1, grabPickup2, intakePickup2, scorePickup2;
//
//    public static double kP = 0.001;
//    public static double kI = 0.0006;
//    public static double kD = 0.0;
//    public static double kF = 0.0;
//
//    public static double kS = 0.0;
//    public static double kV = 0.00042;
//    public static double kA = 0.0;
//    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
//    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
//
//    private boolean shooting = false;
//    private int shootStage = 0;
//    private int shotsFired = 0;
//    private double targetRPM = 2030; //2100
//    private final int ticksPerRevLauncher = 28;
//
//    public void startShooting3() {
//        shooting = true;
//        shootStage = 0;
//        shotsFired = 0;
//        actionTimer.resetTimer();
//    }
//
//    public void updateShooting() {
//        if (!shooting) return;
//
//        double targetTicksPerSec = targetRPM / 60.0 * ticksPerRevLauncher;
//        double measuredTicksPerSec = robot.launcher.getVelocity();
//        double ffOutput = feedforward.calculate(targetTicksPerSec);
//        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);
//        double combinedOutput = Math.max(-1.0, Math.min(1.0, ffOutput + pidOutput));
//        robot.launcher.setPower(combinedOutput);
//
//        double errorRPM = Math.abs(measuredTicksPerSec / ticksPerRevLauncher * 60.0 - targetRPM);
//
//        switch (shootStage) {
//            case 0:
//                if (errorRPM < 75 || actionTimer.getElapsedTimeSeconds() > 4.0) {
//                    shootStage = 1;
//                    actionTimer.resetTimer();
//                }
//                break;
//
//            case 1:
//                double measuredRPM = Math.abs(robot.launcher.getVelocity() / ticksPerRevLauncher * 60.0);
//                double allowedError = 75;
//
//                robot.launcher.setVelocity(targetRPM / 60.0 * ticksPerRevLauncher);
//
//                if (measuredRPM >= targetRPM - allowedError) {
//                    if (actionTimer.getElapsedTimeSeconds() >= 0.07) {
//                        robot.intake.setPower(1);
//                        robot.intakeServo.setPower(1);
//                    }
//                    if (actionTimer.getElapsedTimeSeconds() >= 0.30) {//0.17
//                        robot.intake.setPower(0);
//                        robot.intakeServo.setPower(1);
//                        shotsFired++;
//                        actionTimer.resetTimer();
//
//                        if (shotsFired >= 3) {
//                            shootStage = 2;
//                        }
//                    }
//                } else {
//                    actionTimer.resetTimer();
//                }
//                break;
//
//            case 2:
//                if (errorRPM < 75 && actionTimer.getElapsedTimeSeconds() > 0.40) {//50, 0.3
//                    if (shotsFired < 3) {
//                        shootStage = 1;
//                        actionTimer.resetTimer();
//                    } else {
//                        shootStage = 3;
//                    }
//                }
//                break;
//
//            case 3:
//                robot.launcher.setPower(0);
//                shooting = false;
//                break;
//        }
//    }
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//        scorePreload.setTranslationalConstraint(0.02);
//        scorePreload.setHeadingConstraint(0.02);
//
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//        intakePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, pickup1intakePose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1intakePose.getHeading())
//                .build();
//
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1intakePose, scorePose))
//                .setLinearHeadingInterpolation(pickup1intakePose.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        intakePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, pickup2intakePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2intakePose.getHeading())
//                .build();
//
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2intakePose, scorePose))
//                .setLinearHeadingInterpolation(pickup2intakePose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            // PRELOAD SCORE
//            case 0:
//                follower.followPath(scorePreload, true);
//                setPathState(1);
//                break;
//
//            // SHOOT + GO TO PICKUP 1
//            case 1:
//                if (!follower.isBusy()) {
//                    double headingError = Math.abs(follower.getPose().getHeading() - scorePose.getHeading());
//
//                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
//                        startShooting3();
//                    }
//
//                    if (!shooting && shotsFired >= 3) {
//                        shotsFired = 0;
//                        targetRPM = 0;
//                        follower.followPath(grabPickup1, 0.5, true); //0.7
//                        setPathState(2);
//                    }
//                }
//                break;
//
//            // PICKUP 1
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
//            // UNIVERSAL INTAKE EXIT
////            case 21:
////                if (actionTimer.getElapsedTimeSeconds() < 0.6) {//.25
////                    robot.launcher.setPower(-0.55);
////                    robot.intake.setPower(0);
////                    robot.intakeServo.setPower(1);
////                }
////
////                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
////                    robot.launcher.setPower(0);
////                    robot.intake.setPower(0);
////                    robot.intakeServo.setPower(1);
////                    slowingForIntake = false;
////                    setPathState(nextState);
////                }
////                break;
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
//                    setPathState(nextState); // Transitions to state 3 (Score Pickup 1) or state 5 (Score Pickup 2)
//                }
////                if (actionTimer.getElapsedTimeSeconds() < 0.25) { //0.25
////                    robot.launcher.setPower(-0.55); //-0.2
////                } else {
////                    robot.launcher.setPower(0);
////                    robot.intake.setPower(0);
////                    robot.intakeServo.setPower(1); //0
////
////                    slowingForIntake = false;   // <<< FIX
////
////                    setPathState(nextState);
////                }
////                break;
//
//                // RETURN + SHOOT PICKUP 1
//            case 3:
//                if (!returningToScore) {
//                    follower.followPath(scorePickup1, 0.7, true);
//                    returningToScore = true;
//                }
//
//                if (!follower.isBusy()) {
//                    double headingError = Math.abs(follower.getPose().getHeading() - scorePose.getHeading());
//
//                    if (!shooting && shotsFired == 0 && headingError < Math.toRadians(5)) {
//                        targetRPM = 2100; //2125
//                        startShooting3();
//                    }
//
//                    if (!shooting && shotsFired >= 3) {
//                        returningToScore = false;
//                        shotsFired = 0;
//
//                        follower.followPath(grabPickup2, 0.5, true); //0.7
//                        setPathState(4);
//                    }
//                }
//                break;
//
//            // PICKUP 2
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
//                        targetRPM = 2100; //2125
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
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//        updateShooting();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("shotsFired", shotsFired);
//        telemetry.addData("shootStage", shootStage);
//        telemetry.addData("shooting", shooting);
//
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        robot.init(hardwareMap);
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override public void init_loop() {}
//    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); }
//    @Override public void stop() {}
//}
