package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.Projects.HWMapOld;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "pp")
public class intakePIDTune extends LinearOpMode {

    public HWMapOld robot = new HWMapOld();

    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();
    ElapsedTime reverseTimer = new ElapsedTime();

    boolean reversingLauncher = false;

    // PIDF + Feedforward constants (starting values — tune these)
    // NOTE: For tuning intake velocity, start with kI = 0 and kF = 0.
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    public static double kS = 0.0;
    public static double kV = 0.00042;
    public static double kA = 0.0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

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

        boolean flywheelon = false; // kept from your logic (used to gate reversingLauncher)
        double speed = 1;

        boolean lastUp = false;
        boolean lastMid = false;
        boolean lastDown = false;
        boolean lastX = false;

        boolean bWasPressed = false;
        boolean isIntakeRunning = false;
        boolean intakeFull = false;

        boolean color1 = false;
        boolean color2 = false;

        // For A-button toggle
        boolean lastAState = false;

        // IMPORTANT: ticksPerRev MUST match the motor/gearbox you're using for intake
        // If you're using REV HD Hex (28 CPR at motor) with e.g. 20:1 gearbox => 560.
        // Your old 224 might be wrong. Change this to correct value.
        int ticksPerRev = 224;

        double setpointRPM = 0;

        // Ensure intake has encoder mode set if you want velocity feedback
        robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ColorSensor sensor1 = hardwareMap.get(ColorSensor.class, "sensor1");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        float hsv1[] = {0F, 0F, 0F};
        float hsv2[] = {0F, 0F, 0F};

        final double SCALE_FACTOR = 255;

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            // Update controller objects for live dashboard tuning
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

            // --- Color scale factor init ---
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

            // --- Hue classification (your thresholds) ---
            if (hue1 < 30) {
                telemetry.addData("Color", "Red");
                color1 = false;
            } else if (hue1 < 60) {
                telemetry.addData("Color", "Orange");
                color1 = false;
            } else if (hue1 < 140) {
                telemetry.addData("Color", "Yellow");
                color1 = false;
            } else if (hue1 < 250) { // green
                telemetry.addData("Color", "Green");
                color1 = true;
            } else if (hue1 < 260) {
                telemetry.addData("Color", "Blue");
                color1 = false;
            } else if (hue1 < 270) { // purple
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
            } else if (hue2 < 180) { // green
                telemetry.addData("Color2", "Green");
                color2 = true;
            } else if (hue2 < 200) {
                telemetry.addData("Color2", "Blue");
                color2 = false;
            } else if (hue2 < 250) { // purple
                telemetry.addData("Color2", "Purple");
                color2 = true;
            } else {
                telemetry.addData("Color2", "Red");
                color2 = false;
            }

            // --- Driver control (unchanged) ---
            double y = -gamepad1.left_stick_y * .8;
            double x = gamepad1.left_stick_x * -1.1 * .8;
            double rx = gamepad1.right_stick_x * .8;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower * speed);
            robot.bLeftWheel.setPower(backLeftPower * speed);
            robot.fRightWheel.setPower(frontRightPower * speed);
            robot.bRightWheel.setPower(backRightPower * speed);

            // --- "Launcher RPM Control" (actually intake velocity setpoint) ---
            boolean highSpeed = gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;

            // Rising edge setpoint updates
            if (highSpeed && !lastUp) {
                setpointRPM = 1000;
                flywheelon = true;
            }
            if (midSpeed && !lastMid) {
                setpointRPM = 600;
                flywheelon = true;
            }
            if (lowSpeed && !lastDown) {
                setpointRPM = 300;
                flywheelon = true;
            }
            if (gamepad1.x && !lastX) {
                setpointRPM = 0;
                flywheelon = false;
                pidf.reset();
            }

            // --- A Button: toggle intake (open-loop) ---
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState && !intakeFull) {
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    buttonTimer.reset();
                }
            }
            lastAState = aNow;

            // --- Full detection logic (as you had, but NOT resetting motor here) ---
            if (color1 && color2) {
                if (colorTimer.milliseconds() > 500 && !intakeFull) {
                    intakeFull = true;
                    reversingLauncher = true;
                    reverseTimer.reset();
                    // stop intake toggle
                    isIntakeRunning = false;
                }
            } else {
                // Your original behavior: immediately clears full state
                // (Can cause flicker; leave as-is for now per request.)
                colorTimer.reset();
                intakeFull = false;
            }

            // --- B button: timed intake pulse (open-loop), requires near target RPM ---
            // NOTE: your old condition used measuredRPM vs setpointRPM. We'll keep it.
            double measuredTicksPerSec = robot.intake.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;

            if (gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 100) {
                if (!bWasPressed) {
                    buttonTimer.reset();
                    bWasPressed = true;
                }
                if (buttonTimer.milliseconds() >= 170) {
                    bWasPressed = false; // stop the pulse after time
                }
            } else if (!isIntakeRunning) {
                bWasPressed = false;
            }

            // ===========================
            // SINGLE OWNER MOTOR COMMAND
            // ===========================
            double motorCmd = 0.0;

            // Define a "velocity tune mode": only when one of these is active AND flywheelon is true.
            boolean velocityTuneMode = flywheelon && (setpointRPM != 0);

            // Priority: reversingLauncher > manual reverse > velocity PID tune > b pulse > a toggle > off
            if (reversingLauncher && !flywheelon) {
                // Your existing reversal behavior uses launcher motor, not intake.
                // We'll keep intake stopped here.
                motorCmd = 0.0;
            }
            else if (gamepad1.dpad_down) {
                // Manual reverse: reliable open-loop (fixes your old targetRPM bug)
                motorCmd = -0.6;
            }
            else if (velocityTuneMode) {
                // PID + FF velocity control of intake
                double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;

                // Feedforward (be careful: SimpleMotorFeedforward assumes "volts-ish" units;
                // you're treating it as power. Tune kS/kV accordingly.)
                double ffOutput = feedforward.calculate(targetTicksPerSec);
                double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

                double combinedOutput = ffOutput + pidOutput;
                combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

                // Deadband near 0 to stop twitching
                if (Math.abs(setpointRPM) < 50) {
                    combinedOutput = 0.0;
                    pidf.reset();
                }

                motorCmd = combinedOutput;

                // Telemetry for tuning
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
            }
            else if (bWasPressed) {
                motorCmd = 0.75;
            }
            else if (isIntakeRunning && !intakeFull) {
                motorCmd = 0.25;
            }
            else {
                motorCmd = 0.0;
            }

            // Apply the single decided power
            robot.intake.setPower(motorCmd);

            // Intake servo control: follow intake state
            // (You can tweak this if your servo meaning is inverted.)
            if (motorCmd != 0 && !intakeFull) {
                robot.intakeServo.setPower(1);
            } else {
                robot.intakeServo.setPower(0);
            }

            // --- Reversing launcher behavior (your original logic) ---
            if (reversingLauncher && flywheelon == false) {
                robot.launcher.setPower(-0.7);
                robot.intakeServo.setPower(0);
                if (reverseTimer.milliseconds() >= 500) {
                    reversingLauncher = false;
                    robot.intakeServo.setPower(0);
                    robot.launcher.setPower(0);
                }
            }

            telemetry.addData("reversingLauncher", reversingLauncher);
            telemetry.addData("flywheelon", flywheelon);

            telemetry.addData("Clear", sensor1.alpha());
            telemetry.addData("Red  ", sensor1.red());
            telemetry.addData("Green", sensor1.green());
            telemetry.addData("Blue ", sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            // store previous D-pad states
            lastUp = highSpeed;
            lastMid = midSpeed;
            lastDown = lowSpeed;
            lastX = gamepad1.x;
        }
    }
}
