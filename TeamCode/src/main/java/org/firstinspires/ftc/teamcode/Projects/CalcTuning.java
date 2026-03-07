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
import com.qualcomm.robotcore.util.Range;
import android.util.Size;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;


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
    public static double kP = 0.006;


    public static double kI = 0.04;
    public static double kD = 0.00008;
    public static double kF = 0.00042;
    public static double IP = 0.0005;




    public static double II = 0;
    public static double ID = 0;
    public static double IF = 0.000048;
    final double TICKS_PER_REV_INTAKE = 101.08;
    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point


    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 1;


    public static double IS = 0.0;
    public static double IV = 0.0;
    public static double IA = 0.2;
    PIDFController Intakepidf = new PIDFController(IP, II, ID, IF);
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    SimpleMotorFeedforward feedforwardIntake = new SimpleMotorFeedforward(IS, IV, IA);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // --- Vision setup ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(100);


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


        double ticksPerRevIntake = 101.08;


        double setpointRPMIntake = 0;


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
            Intakepidf.setPIDF(IP,II,ID,IF);
            feedforwardIntake = new SimpleMotorFeedforward(IS,IV,IA);
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
            if (gamepad1.right_bumper){
                setpointRPM = 1700;
                flywheelon = true;
            }


            // --- A Button -- toggle intake and shuts off when full
            boolean aNow = gamepad1.a;
            if (aNow && !lastAState) {
                // just pressed
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    setpointRPMIntake = 1450;
                    buttonTimer.reset();
                } else {
                    setpointRPMIntake = 0;
                    robot.intake.setPower(0);
                }
            }




            lastAState = aNow;




            // Measurements in ticks/sec
            double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
            double measuredTicksPerSec = robot.flywheel.getVelocity();
            double measuredRPM = measuredTicksPerSec /60 * ticksPerRev;


            double targetTicksPerSecIntake = setpointRPMIntake/60 * ticksPerRevIntake;
            double measuredTicksPerSecIntake = robot.intake.getVelocity();


            // Feedforward baseline (returns value in same "command" units as gains —
            // we've chosen gains so this approximates motor power)
            double ffOutput = feedforward.calculate(targetTicksPerSec);


            // PIDF returns correction. Give it the measurement and target (also ticks/sec).
            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);


            // Combine and clamp to motor power range [-1, 1]
            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));


            double ffOutputIntake = feedforwardIntake.calculate(targetTicksPerSecIntake);
            double pidOutputIntake = Intakepidf.calculate(measuredTicksPerSecIntake,targetTicksPerSecIntake);
            double combinedOutputIntake = ffOutputIntake + pidOutputIntake;
            combinedOutputIntake = Math.max(-1.0, Math.min(1.0, combinedOutputIntake));


            // If the driver pressed D-pad (we want launcher behavior), apply combined power.
            // If the player pressed 'x' or dpad_down, those override below.
            robot.intake.setPower(combinedOutputIntake);
            robot.flywheel.setPower(combinedOutput);










            // --- Dpad down: reverse intake & launcher negative (manual) ---


            // --- B button: timed intake pulse ---
//            if (gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 100) {
//                if (!bWasPressed) {
//                    buttonTimer.reset();
//                    robot.intake.setPower(0.75);
//                    bWasPressed = true;
//                }
//                if (buttonTimer.milliseconds() >= 170) {
//                    robot.intake.setPower(0);
//                }
//            } else if (!isIntakeRunning) {
//                bWasPressed = false;
//                robot.intake.setPower(0);
//            }
            LLStatus status = limelight.getStatus();




            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {




                // Access general information
                Pose3D botpose = result.getBotpose();
                double distance = getDistance(result.getTy());
                double tx = result.getTx();
                double txnc = result.getTxNC();
                double ty = result.getTy();
                double tync = result.getTyNC();
                telemetry.addLine("eversonisgoat robot sees apritag");


                if(gamepad1.left_bumper){
                    setpointRPM = 10*(0.026218 * Math.pow(distance, 3) - 2.53637 * Math.pow(distance, 2) + 82.8973 * distance - 672.92205);
                }




                //shootCODE






                // telemetry.addData("balls are in?", filled);
                //shootEND
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int apriltagID = fr.getFiducialId();
                    if (apriltagID == 20) {
                        telemetry.addLine("eversonisgoat robot sees correct apritag");
                    }
                }
            }


            // --- Telemetry for tuning ---
            telemetry.addData("Setpoint RPM", setpointRPM);
            packet.put("Setpoint RPM", setpointRPM);
            telemetry.addData("Measured RPM", "%.1f", measuredRPM);
            packet.put("Measured RPM", measuredRPM);


            telemetry.addData("Distance", getDistance(result.getTy()));
            packet.put("Distance", getDistance(result.getTa()));


            telemetry.addData("Setpoint RPM Intake", setpointRPMIntake);
            packet.put("Setpoint RPM Intake", setpointRPMIntake);
            telemetry.addData("Measured RPM Intake", "%.1f", measuredTicksPerSecIntake);
            packet.put("Measured RPM Intake", measuredTicksPerSecIntake);




            // --- AprilTag Centering (Y button) ---
// While the robot is looking at the tag once, it will toggle intake off
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();


            // store previous D-pad states
            lastUp = upSpeed;
            lastDown = downSpeed;
        }
    }
    public double getDistance(double ty) {
        double limelightMountAngle = 20.0;
        double limelightHeightInches = 14;
        double goalHeightInches = 29.5;
        double angleToGoal = limelightMountAngle + ty;
        double angleRadians = angleToGoal * (Math.PI / 180.0);


        return (goalHeightInches - limelightHeightInches) / Math.tan(angleRadians);
    }
}









