
package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
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
@TeleOp(name = "ILT teleop")
public class ILTTELEOP extends LinearOpMode {

//LauncherPID:
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
    final double MAX_DEGREES = 90.0;

    final double MIN_POWER_TO_MOVE = 0.05;
    final double BEARING_TOLERANCE = 1.5;    // degrees

    @Override
    public void runOpMode() throws InterruptedException
    {

//Variables:
        boolean lastAState = false;
        boolean intakeFull = false;
        boolean isIntakeRunning = false;
        boolean color1 = false;
        boolean color2 = false;
        boolean color3 = false;
        float hsv1[] = {0F, 0F, 0F};
        float hsv2[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        boolean lastUp = false;
        boolean lastMid = false;
        boolean lastDown = false;
        boolean lastX = false;
        double setpointRPM = 0;
        boolean flywheelon = false;
        int ticksPerRev = 28;

        robot.init(hardwareMap);

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

//DriveCode:
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = gamepad1.right_stick_x;
            double speed = .7;


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
            if (gamepad1.x && !lastX){
                setpointRPM = 0;
                flywheelon = false;
            }
            double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
            double measuredTicksPerSec = robot.flywheel.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;

            // Feedforward baseline (returns value in same "command" units as gains —
            // we've chosen gains so this approximates motor power)
            double ffOutput = feedforward.calculate(targetTicksPerSec);

            // PIDF returns correction. Give it the measurement and target (also ticks/sec).
            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            // Combine and clamp to motor power range [-1, 1]
            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            // If the driver pressed D-pad (we want launcher behavior), apply combined power.
            // If the player pressed 'x' or dpad_down, those override below.
            robot.flywheel.setPower(combinedOutput);
//LauncherCodeEND
//IntakeCode:
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
            float hue1 = hsv1[0];
            float hue2 = hsv2[0];

    if (hue1 < 30) {
        telemetry.addData("Color", "Red");
        color1 = false;
    } else if (hue1 < 60) {
        telemetry.addData("Color", "Orange");
        color1 = false;
    } else if (hue1 < 140) {
        telemetry.addData("Color", "Yellow");
        color1 = false;

    } else if (hue1 < 250) { //green --> 160
        telemetry.addData("Color", "Green");
        color1 = true;
    } else if (hue1 < 260) {
        telemetry.addData("Color", "Blue");
        color1 = false;

    } else if (hue1 < 270) { //purple --> 230-250
        telemetry.addData("Color", "Purple");
        color1 = true;
    } else {
        telemetry.addData("Color", "Red");
        color1 = false;
    }

    if (hue2 < 30) {
        telemetry.addData("Color2", "Red");
        color2 = false;
    } else if (hue2 < 60) {
        telemetry.addData("Color2", "Orange");
        color2 = false;
    } else if (hue2 < 140) {
        telemetry.addData("Color2", "Yellow");
        color2 = false;
    } else if (hue2 < 180) { //green --> 160
        telemetry.addData("Color2", "Green");
        color2 = true;
    } else if (hue2 < 200) {
        telemetry.addData("Color2", "Blue");
        color2 = false;
    } else if (hue2 < 250) { //purple --> 230-250
        telemetry.addData("Color2", "Purple");
        color2 = true;
    } else {
        telemetry.addData("Color2", "Red");
        color2 = false;
    }
//if you don't want to go to winter formal with someone, get them to set up Justin with a date before you go
    if (hue2 < 30) {
        telemetry.addData("Color2", "Red");
        color3 = false;
    } else if (hue2 < 60) {
        telemetry.addData("Color2", "Orange");
        color3 = false;
    } else if (hue2 < 140) {
        telemetry.addData("Color2", "Yellow");
        color3 = false;
    } else if (hue2 < 180) { //green --> 160
        telemetry.addData("Color2", "Green");
        color3 = true;
    } else if (hue2 < 200) {
        telemetry.addData("Color2", "Blue");
        color3 = false;
    } else if (hue2 < 250) { //purple --> 230-250
        telemetry.addData("Color2", "Purple");
        color3 = true;
    } else {
        telemetry.addData("Color2", "Red");
        color3 = false;
    }

    boolean aNow = gamepad1.a;
    if (aNow && !lastAState && !intakeFull) {
        // just pressed
        isIntakeRunning = !isIntakeRunning;
        if (isIntakeRunning) {
            robot.intake.setPower(0.25);
            buttonTimer.reset();
        } else {
            robot.intake.setPower(0);
        }
    }



            lastAState = aNow;



            if (color1 && color2&color3){
                if (colorTimer.milliseconds() > 500 && !intakeFull){
                    robot.intake.setPower(0);
                    intakeFull = true;
                }
            }
            else{
                colorTimer.reset();
                intakeFull = false;
            }
//IntakeCodeEND
//lift:
            if(gamepad1.right_bumper){
                robot.lift.setTargetPosition(10);

            }
//liftEND
//TrackingCode:
            LLStatus status = limelight.getStatus();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double tx = result.getTx();
                double txnc = result.getTxNC();
                double ty = result.getTy();
                double tync = result.getTyNC();


                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());


                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int apriltagID = fr.getFiducialId();
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    if(apriltagID == 20) {

                        int encoderTicks = robot.turret.getCurrentPosition();
                        double turretDegrees = (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;

                        double angleError = -tx;

                        double motorPower = 0.0;

                        if (Math.abs(angleError) > BEARING_TOLERANCE) {
                            motorPower = angleError * kP;

                            if (Math.abs(motorPower) < MIN_POWER_TO_MOVE) {
                                motorPower = Math.signum(motorPower) * MIN_POWER_TO_MOVE;
                            }
                        }


                        if ((turretDegrees <= -MAX_DEGREES && motorPower < 0) ||
                                (turretDegrees >= MAX_DEGREES && motorPower > 0)) {
                            motorPower = 0;
                        }

                        motorPower = Range.clip(motorPower, -1.0, 1.0);
                        robot.turret.setPower(motorPower);

                        telemetry.addData("Turret Angle (deg)", "%.2f", turretDegrees);
//                telemetry.addData("Desired Angle (deg)", "%.2f", "desiredTurretAngle");
                        telemetry.addData("Angle Error (deg)", "%.2f", angleError);
                        telemetry.addData("Motor Power", "%.2f", motorPower);
                        telemetry.addData("Encoder Ticks", encoderTicks);
                        telemetry.addData("Target X", tx);
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
                robot.turret.setPower(0);
            }
//TrackingCodeEND

            telemetry.update();
        }
        limelight.stop();
    }
}