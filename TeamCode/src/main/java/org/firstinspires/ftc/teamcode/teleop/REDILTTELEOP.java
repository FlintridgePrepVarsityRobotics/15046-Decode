
package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Projects.newHWmap;

import java.util.List;



@Config
@TeleOp(name = "redILT teleop")
public class REDILTTELEOP extends LinearOpMode {
    PIDController turretpid = new PIDController(TP, TI, TD);
    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;
// PIDF + Feedforward constants (starting values — tune these)
// These gains are chosen so PIDF+FF outputs a motor power in [-1,1].

public static double kP = 0.0125;

    public static double kI = 0.00015;
    public static double kD = 0.00000005;
    public static double kF = 0.0004208;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
//LauncherPIDEND
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();

    private Limelight3A limelight;
    public newHWmap robot = new newHWmap();

    final double TICKS_PER_REV = 294.0;      // GoBilda 5202/5203
    final double GEAR_RATIO = 0.3953;        // 34 / 86
    final double MAX_DEGREES = 70;

    final double MIN_POWER_TO_MOVE = 0.05;
    final double BEARING_TOLERANCE = 7.5;    // degrees
    final double TICKS_PER_REV_INTAKE = 146.44;


    @Override
    public void runOpMode() throws InterruptedException
    {

//variables:
        boolean lastAState = false;
        boolean intakeFull = false;
        boolean isIntakeRunning = false;
        boolean color1 = false;
        boolean color2 = false;
        boolean color3 = false;
        float hsv1[] = {0F, 0F, 0F};
        float hsv2[] = {0F, 0F, 0F};
        float hsv3[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        boolean lastUp = false;
        boolean lastMid = false;
        boolean lastDown = false;
        boolean lastX = false;
        double setpointRPM = 0;
        boolean flywheelon = false;
        int ticksPerRev = 28;
        boolean filled = false;

        robot.init(hardwareMap);

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(5);


        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

//        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

//DriveCode:
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = gamepad1.right_stick_x;
            double speed = 1;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);
//DriveCodeEND

//FlywheelCode:
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

            boolean highSpeed = gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;



            if (highSpeed && !lastUp){
                setpointRPM = 2400;
                flywheelon = true;
            }
            if (midSpeed && !lastMid){
                setpointRPM = 2100;
                flywheelon = true;
            }
            if (lowSpeed && !lastDown){
                setpointRPM = 1800;
                flywheelon = true;
            }
            if (gamepad1.dpad_down){
                setpointRPM = 0;
            }
            if(gamepad1.x){
                robot.intake.setVelocity(0);
            }

            double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
            double measuredTicksPerSec = robot.flywheel.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;
            telemetry.addData("setpointRPM", (setpointRPM));
            telemetry.addData("measuredRPM", measuredRPM);
            telemetry.update();

            double ffOutput = feedforward.calculate(targetTicksPerSec);

            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            robot.flywheel.setPower(combinedOutput);
//LauncherCodeEND
//IntakeCode:
//            Color.RGBToHSV(
//                    (int) (robot.sensor1.red() * SCALE_FACTOR),
//                    (int) (robot.sensor1.green() * SCALE_FACTOR),
//                    (int) (robot.sensor1.blue() * SCALE_FACTOR),
//                    hsv1
//            );
//
//            Color.RGBToHSV(
//                    (int) (robot.sensor2.red() * SCALE_FACTOR),
//                    (int) (robot.sensor2.green() * SCALE_FACTOR),
//                    (int) (robot.sensor2.blue() * SCALE_FACTOR),
//                    hsv2
//            );
//            Color.RGBToHSV(
//                    (int) (robot.sensor3.red() * SCALE_FACTOR),
//                    (int) (robot.sensor3.green() * SCALE_FACTOR),
//                    (int) (robot.sensor3.blue() * SCALE_FACTOR),
//                    hsv3
//            );
//            float hue1 = hsv1[0];
//            float hue2 = hsv2[0];
//            float hue3 = hsv3[0];
//
//    if (hue1 < 30) {
////        telemetry.addData("Color", "Red");
//        color1 = false;
//    } else if (hue1 < 60) {
////        telemetry.addData("Color", "Orange");
//        color1 = false;
//    } else if (hue1 < 140) {
////        telemetry.addData("Color", "Yellow");
//        color1 = false;
//
//    } else if (hue1 < 250) { //green --> 160
////        telemetry.addData("Color", "Green");
//        color1 = true;
//    } else if (hue1 < 260) {
////        telemetry.addData("Color", "Blue");
//        color1 = false;
//
//    } else if (hue1 < 270) { //purple --> 230-250
////        telemetry.addData("Color", "Purple");
//        color1 = true;
//    } else {
////        telemetry.addData("Color", "Red");
//        color1 = false;
//    }
//
//    if (hue2 < 30) {
////        telemetry.addData("Color2", "Red");
//        color2 = false;
//    } else if (hue2 < 60) {
////        telemetry.addData("Color2", "Orange");
//        color2 = false;
//    } else if (hue2 < 140) {
////        telemetry.addData("Color2", "Yellow");
//        color2 = false;
//    } else if (hue2 < 180) { //green --> 160
////        telemetry.addData("Color2", "Green");
//        color2 = true;
//    } else if (hue2 < 200) {
////        telemetry.addData("Color2", "Blue");
//        color2 = false;
//    } else if (hue2 < 250) { //purple --> 230-250
////        telemetry.addData("Color2", "Purple");
//        color2 = true;
//    } else {
////        telemetry.addData("Color2", "Red");
//        color2 = false;
//    }
////if you don't want to go to winter formal with someone, get them to set up Justin with a date before you go
//    if (hue2 < 30) {
////        telemetry.addData("Color2", "Red");
//        color3 = false;
//    } else if (hue2 < 60) {
////        telemetry.addData("Color2", "Orange");
//        color3 = false;
//    } else if (hue2 < 140) {
////        telemetry.addData("Color2", "Yellow");
//        color3 = false;
//    } else if (hue2 < 180) { //green --> 160
////        telemetry.addData("Color2", "Green");
//        color3 = true;
//    } else if (hue2 < 200) {
////        telemetry.addData("Color2", "Blue");
//        color3 = false;
//    } else if (hue2 < 250) { //purple --> 230-250
////        telemetry.addData("Color2", "Purple");
//        color3 = true;
//    } else {
////        telemetry.addData("Color2", "Red");
//        color3 = false;
//    }

            // ... inside the while(opModeIsActive) loop ...

// --- SENSOR READING ---
            // ----------------------------------------------------------------------
// 1. SENSOR & COLOR READING
            Color.RGBToHSV((int) (robot.sensor1.red() * SCALE_FACTOR), (int) (robot.sensor1.green() * SCALE_FACTOR), (int) (robot.sensor1.blue() * SCALE_FACTOR), hsv1);
            Color.RGBToHSV((int) (robot.sensor2.red() * SCALE_FACTOR), (int) (robot.sensor2.green() * SCALE_FACTOR), (int) (robot.sensor2.blue() * SCALE_FACTOR), hsv2);
            Color.RGBToHSV((int) (robot.sensor3.red() * SCALE_FACTOR), (int) (robot.sensor3.green() * SCALE_FACTOR), (int) (robot.sensor3.blue() * SCALE_FACTOR), hsv3);

            String s1 = classifyColor(hsv1);
            String s2 = classifyColor(hsv2);
            String s3 = classifyColor(hsv3);

            boolean c1 = !s1.equals("EMPTY") && !s1.equals("UNKNOWN");
            boolean c2 = !s2.equals("EMPTY") && !s2.equals("UNKNOWN");
            boolean c3 = !s3.equals("EMPTY") && !s3.equals("UNKNOWN");

            telemetry.addData("Intake", "[%s] [%s] [%s]", s1, s2, s3);

            if (c1 && c2 && c3) {
                if (colorTimer.seconds() > 0.3) {
                    intakeFull = true;
                }
            } else {
                colorTimer.reset();
                intakeFull = false;
            }

// 3. HANDLE DRIVER INPUT (TOGGLE A)
//            boolean aNow = gamepad1.a;
//            if (aNow && !lastAState) {
//                isIntakeRunning = !isIntakeRunning;
//            }
//            lastAState = aNow;
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState && !intakeFull) {    // 500*ticksperrev is #ofrevolutions we need per min
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 500 / 60);
//            robot.intake.setVelocity(TICKS_PER_REV_INTAKE*1100/60); //test code
                    robot.shootServo.setPosition(0.5);
                    buttonTimer.reset();
                } else {
                    robot.intake.setPower(0);
                }
            }


// 4. AUTO-STOP LOGIC
            if (intakeFull && isIntakeRunning) {
                isIntakeRunning = false;
            }

// Priority 1: SHOOTING (Button B)
            boolean isShooting = gamepad1.b && (Math.abs(measuredRPM - setpointRPM) <= 100);

            if (isShooting) {
                robot.shootServo.setPosition(0);
                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1100 / 60);

                filled = false;

            }
            else if (isIntakeRunning) {
                robot.shootServo.setPosition(0.5);
                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 500 / 60);
            }
            else {
                robot.intake.setPower(0);
                robot.shootServo.setPosition(0);
            }

            if (intakeFull) filled = true;
            telemetry.addData("System Full?", filled);
//IntakeCodeEND
//lift:
//            if(gamepad1.right_bumper){
//                robot.lift.setTargetPosition(10);
//
//            }
//liftEND
//TrackingCode:
            LLStatus status = limelight.getStatus();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double distance = getdistance(result.getTa());
                double tx = result.getTx();
                double txnc = result.getTxNC();
                double ty = result.getTy();
                double tync = result.getTyNC();

 //shootCODE

                if(color1 || color2 || color3){
                    filled = true;

                }

                telemetry.addData("balls are in?", filled);
                if(gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 100){
                    robot.shootServo.setPosition(0);
                    robot.intake.setVelocity(TICKS_PER_REV_INTAKE*1100/60);
                    }

                //shootEND

                telemetry.addData("distance", distance);
                telemetry.addData("tx", result.getTx());
//                telemetry.addData("txnc", result.getTxNC());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("target area", result.getTa());

                telemetry.addData("Botpose", botpose.toString());
                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int apriltagID = fr.getFiducialId();

//
//                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    if(apriltagID == 24) {
                        double targetX = fr.getTargetXDegrees();
                        double turretpidOutput = turretpid.calculate(0, targetX);

                        double turretfeedforward = 0;
                        double AngleError = -tx;


                        if (Math.abs(turretpidOutput) > 0.01) {
                            turretfeedforward = Math.signum(turretpidOutput) * MIN_POWER_TO_MOVE;
                        }

                        double motorPower;
                        if(Math.abs(AngleError) < BEARING_TOLERANCE)
                            motorPower = 0;
                        else
                            motorPower = -(turretpidOutput + turretfeedforward);

                        int encoderTicks = robot.turret.getCurrentPosition();
                        double turretDegrees = (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;

                        if ((turretDegrees >= MAX_DEGREES && motorPower > 0) ||
                                (turretDegrees <= -MAX_DEGREES && motorPower < 0)) {
                            motorPower = 0;
                        }

                        robot.turret.setPower(motorPower);

                        telemetry.addData("Limelight Target X", targetX);
                        telemetry.addData("Final Power", motorPower);
                        telemetry.addData("Angle", turretDegrees);
                    }
                }
            } else if (gamepad1.y) {
        int encoderTicks = robot.turret.getCurrentPosition();
        double currentDegrees = (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;

        if (currentDegrees > 2) {
            robot.turret.setPower(-0.3);
        }
        else if (currentDegrees < -2) {
            robot.turret.setPower(0.3);
        }
        else {
            robot.turret.setPower(0);
        }

        telemetry.addData("Mode", "Manual Reset");
        telemetry.addData("Angle", currentDegrees);
    }
            else {
                telemetry.addData("Limelight", "No data available");
                robot.turret.setPower(0);
            }
//TrackingCodeEND

            telemetry.update();
        }

        limelight.stop();
    }
    public double getdistance(double ta){
        double scale = 10;
        double distance = scale/ta;
return(distance);
    }

    public double getCombinedOutput (double setpointRPM){
        int ticksPerRev = 28;
        double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
        double measuredTicksPerSec = robot.flywheel.getVelocity();
        double ffOutput = feedforward.calculate(targetTicksPerSec);
        double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);
        double combinedOutput = ffOutput + pidOutput;
        double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;
        combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));
        return(combinedOutput);
    }
    private String classifyColor(float[] hsv) {
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < 0.3 || val < 0.3) {
            return "EMPTY";
        }

        if (hue >= 110 && hue <= 170) {
            return "GREEN";
        }

        if (hue >= 250 && hue <= 320) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
}