package org.firstinspires.ftc.teamcode.Projects;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Config
@TeleOp(name = "CalcTuning")
public class CalcTuning extends LinearOpMode {

    public newHWmap robot = new newHWmap();
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();
    private Limelight3A limelight;

    boolean reversingLauncher = false;

    // PIDF + Feedforward constants (starting values — tune these)
    // These gains are chosen so PIDF+FF outputs a motor power in [-1,1].
    public static double kP = 0.002;

    public static double kI = 0.0;
    public static double kD = 0.00025;
    public static double kF = 0.00042;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // --- Vision setup ---

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(5);

        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.update();

        int frameWidth = 640;
        int centerX = frameWidth / 2;
        int tolerance = 50; // pixels within which the tag is centered
        boolean flywheelon = false;
        double speed = 1;
        boolean lastUp = false;
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
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColorSensor sensor1;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
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

            // color scale factor init


            boolean tagCentered = false;



            // --- Driver control ---
            double y = -gamepad1.left_stick_y*.7;
            double x = gamepad1.left_stick_x * -1.1*.7;
            double rx = gamepad1.right_stick_x*.7;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);

            LLResult result = limelight.getLatestResult();

            // --- Launcher RPM Control ---

            boolean upSpeed = gamepad1.dpad_up;

            boolean downSpeed = gamepad1.dpad_down;

            // Update setpoint only when a D-pad button is newly pressed (rising edge),
            // so you don't keep re-setting it each loop.
            if (upSpeed && !lastUp){
                setpointRPM += 50;
                flywheelon = true;
            }
            if (downSpeed && !lastDown){
                setpointRPM -= 50;
                flywheelon = true;
            }

            if (gamepad1.x && !lastX){
                setpointRPM = 0;
                flywheelon = false;
            }

            // Measurements in ticks/sec
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



            // --- Dpad down: reverse intake & launcher negative (manual) ---

            // --- A Button -- toggle intake and shuts off when full
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState) {
                // just pressed
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    robot.intake.setPower(0.8);
                    buttonTimer.reset();
                } else {
                    robot.intake.setPower(0);
                }
            }


            lastAState = aNow;

            telemetry.update();
            // --- B button: timed intake pulse ---
            if (gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 100) {
                if (!bWasPressed) {
                    buttonTimer.reset();
                    robot.intake.setPower(0.75);
                    bWasPressed = true;
                }
                if (buttonTimer.milliseconds() >= 170) {
                    robot.intake.setPower(0);
                }
            } else if (!isIntakeRunning) {
                bWasPressed = false;
                robot.intake.setPower(0);
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
            telemetry.addData("Distance", getdistance(result.getTa()));
            packet.put("Distance", getdistance(result.getTa()));

            telemetry.addData("Clear", sensor1.alpha());
            telemetry.addData("Red  ", sensor1.red());
            telemetry.addData("Green", sensor1.green());
            telemetry.addData("Blue ", sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);

            // --- AprilTag Centering (Y button) ---
// While the robot is looking at the tag once, it will toggle intake off
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            // store previous D-pad states
            lastUp = upSpeed;
            lastDown = downSpeed;
        }
    }
    public double getdistance(double ta){
        double scale = 10;
        return(scale/ta);
    }
}




