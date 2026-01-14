
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Projects.newHWmap;

@Config
@TeleOp(name = "turretPIDTune")
public class turretPIDTune extends LinearOpMode {

    public newHWmap robot = new newHWmap();


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

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // --- Vision setup ---

        int frameWidth = 640;
        int centerX = frameWidth / 2;
        int tolerance = 50; // pixels within which the tag is centered
        boolean flywheelon = false;
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
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        // get a reference to the distance sensor that shares the same name

        // hsvValues is an array that will hold the hue, saturation, and value information.


        // values is a reference to the hsvValues array.

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.

        waitForStart();

        while (opModeIsActive()) {
            // Creating obj for PID Tuning
            TelemetryPacket packet = new TelemetryPacket();
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

            // color scale factor init




            boolean tagCentered = false;



            // --- Driver control ---




            // --- Launcher RPM Control ---
            boolean highSpeed = gamepad1.dpad_right;
            boolean midSpeed = gamepad1.dpad_up;
            boolean lowSpeed = gamepad1.dpad_left;

            // Update setpoint only when a D-pad button is newly pressed (rising edge),
            // so you don't keep re-setting it each loop.
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


            // --- AprilTag Centering (Y button) ---

// While the robot is looking at the tag once, it will toggle intake off
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            // store previous D-pad states
            lastUp = highSpeed;
            lastMid = midSpeed;
            lastDown = lowSpeed;
        }
    }
}
