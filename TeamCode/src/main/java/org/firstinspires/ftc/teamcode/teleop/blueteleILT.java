
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Projects.newHWmap;

import java.util.List;



@Config
@TeleOp(name = "blueILT teleop")
public class blueteleILT extends LinearOpMode {
    private boolean intakeServoRunning = false;
    private double intakeServoStartTime = 0.0;

    private static final double INTAKE_SERVO_RUN_TIME = 1.0;

    //Eversons very good stuff
    private boolean returningToScore = false;
    private boolean slowingForIntake = false;




    private ElapsedTime intakeTimer = new ElapsedTime();



    //Everson very good stuff
    public void updateIntakeServo() {
        if (!intakeServoRunning) return;

        double now = intakeTimer.time();
        if (now - intakeServoStartTime >= INTAKE_SERVO_RUN_TIME) {
            robot.intake.setPower(0.0);
            intakeServoRunning = false;
        }
    }
    private NormalizedColorSensor test_color;
    private NormalizedColorSensor test_color2;
    private NormalizedColorSensor test_color3;
    PIDController turretpid = new PIDController(TP, TI, TD);
    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;

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
    //LauncherPIDEND
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();

    private Limelight3A limelight;
    public newHWmap robot = new newHWmap();

    final double TICKS_PER_REV = 294.0;      // GoBilda 5202/5203
    final double GEAR_RATIO = 0.3953;        // 34 / 86
    final double MAX_DEGREES = 70;

    final double MIN_POWER_TO_MOVE = 0.05;
    final double BEARING_TOLERANCE = 2;    // degrees
    final double TICKS_PER_REV_INTAKE = 146.44;

    double targetTicksPerSec = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {

//variables:
        boolean centered  = false;
        boolean lastAState = false;
        boolean intakeFull = false;
        boolean isIntakeRunning = false;
        boolean allowUp = true;
        int poo = 0;
//        float hsv1[] = {0F, 0F, 0F};
//        float hsv2[] = {0F, 0F, 0F};
//        float hsv3[] = {0F, 0F, 0F};

        test_color = hardwareMap.get(NormalizedColorSensor.class, "sensor1");
        test_color.setGain(20);
        test_color2 = hardwareMap.get(NormalizedColorSensor.class, "sensor2");
        test_color2.setGain(20);
        test_color3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3");
        test_color3.setGain(20);
//        color1 = hardwareMap.get(NormalizedColorSensor.class, "sensor1");
//        color2 = hardwareMap.get(NormalizedColorSensor.class, "sensor2");
//        color3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3");
        final double SCALE_FACTOR = 255;

        int hsvcolor1 = 0;
        int hsvcolor2 = 0;
        int hsvcolor3 = 0;

        boolean sense1 = false;
        boolean sense2 = false;
        boolean sense3 = false;

        boolean lastBState = false;
        boolean intakeAfterShot = false;

        ElapsedTime intakeReleaseTimer = new ElapsedTime();


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
            telemetry.addData("elapsedtimer",intakeTimer.time());

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
//liftCode:
//            if(gamepad1.right_bumper){
//                allowUp = !allowUp;
//            }
//            telemetry.addData("toggle lift:",allowUp);
//
            if(gamepad1.right_bumper){
                allowUp = true;

            }
            if(gamepad1.left_bumper){
                allowUp = false;

            }
//FlywheelCode:
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);



            if(gamepad1.x){
                robot.intake.setVelocity(-0.5);
            }
            double measuredTicksPerSec = robot.flywheel.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;
//            telemetry.addData("setpointRPM", (setpointRPM));
//            telemetry.addData("measuredRPM", measuredRPM);
//            telemetry.update();

            double ffOutput = feedforward.calculate(targetTicksPerSec);

            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            robot.flywheel.setPower(combinedOutput);
//LauncherCodeEND
//IntakeCode:



            NormalizedRGBA colors = test_color.getNormalizedColors();
            telemetry.addData("Light Detected1", ((OpticalDistanceSensor) test_color).getLightDetected());
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));

            NormalizedRGBA colors2 = test_color2.getNormalizedColors();
            telemetry.addData("Light Detected2", ((OpticalDistanceSensor) test_color2).getLightDetected());
            telemetry.addData("Red", "%.3f", colors2.red);
            telemetry.addData("Green", "%.3f", colors2.green);
            telemetry.addData("Blue", "%.3f", colors2.blue);
            telemetry.addData("Hue", JavaUtil.colorToHue(colors2.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors2.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors2.toColor()));

            NormalizedRGBA colors3 = test_color2.getNormalizedColors();
            telemetry.addData("Light Detected3", ((OpticalDistanceSensor) test_color3).getLightDetected());
            telemetry.addData("Red", "%.3f", colors3.red);
            telemetry.addData("Green", "%.3f", colors3.green);
            telemetry.addData("Blue", "%.3f", colors3.blue);
            telemetry.addData("Hue", JavaUtil.colorToHue(colors3.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors3.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors3.toColor()));

            if(((OpticalDistanceSensor) test_color).getLightDetected()>.05){
                telemetry.addData("sensor1 is full twin","Everson is the goat");
                sense1 = true;
            }else{
                sense1 = false;
            }

            if(((OpticalDistanceSensor) test_color2).getLightDetected()>.06){
                telemetry.addData("sensor2 is full twin","Everson is the goat");
                sense2 = true;
            }else{
                sense2 = false;
            }

            if(((OpticalDistanceSensor) test_color3).getLightDetected()>.06){
                telemetry.addData("sensor3 is full twin","Everson is the goat");
                sense3 = true;
            }else{
                sense3 = false;
            }


            if (sense1 && sense2 && sense3) {
                if (colorTimer.seconds() > .7) {
                    intakeFull = true;
                    telemetry.addData("intake is full twin","Everson is the goat");
                }
            } else {
                colorTimer.reset();
                intakeFull = false;
            }


            boolean aNow = gamepad1.a;
            if (aNow && !lastAState && !intakeFull && buttonTimer.seconds() > 0.6) {    // 500*ticksperrev is #ofrevolutions we need per min
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    robot.shootServo.setPosition(0.5);
                    robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 800 / 60);
//            robot.intake.setVelocity(TICKS_PER_REV_INTAKE*1100/60); //test code
                    buttonTimer.reset();
                } else {
                    robot.intake.setVelocity(0);
                    robot.shootServo.setPosition(0);
                }
            }else{
                robot.intake.setVelocity(0);
            }


// 4. AUTO-STOP LOGIC
            if (intakeFull && isIntakeRunning) {
                isIntakeRunning = false;
            }

// Priority 1: SHOOTING (Button B)
//            if(gamepad1.b){
//                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1100 / 60);
//            }

            boolean bNow = gamepad1.b;

// 🔥 B was released
            if (!bNow && lastBState) {
                // 1️⃣ Close shoot servo
                robot.shootServo.setPosition(0.5);

                // 2️⃣ Start intake
                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1100 / 60);

                intakeReleaseTimer.reset();
                intakeAfterShot = true;
            }

            lastBState = bNow;

            if (intakeAfterShot) {
                if (intakeReleaseTimer.seconds() >= 1.0) {
                    robot.intake.setVelocity(0);

                    intakeAfterShot = false;
                }
            }



//            boolean isShooting = gamepad1.b;
//            if (isShooting) {
//                robot.shootServo.setPosition(0);
//                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1100 / 60);
//                telemetry.addData("Everson","is the goat 1");
//                if(allowUp) {
//                    telemetry.addData("Everson","is the goat 2 ");
//                    robot.lift.setTargetPosition(-15);
//                    robot.lift.setPower(1);
//                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
////
//
//                filled = false;
//
//            }
//            else if (isIntakeRunning) {
//                robot.shootServo.setPosition(0.5);
//                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 500 / 60);
//            }
//            else {
//                robot.intake.setPower(0);
//                robot.lift.setTargetPosition(1);
//                robot.lift.setPower(-1);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

            if (intakeFull) filled = true;
            // telemetry.addData("System Full?", filled);
//IntakeCodeEND
//lift:
            if(gamepad1.right_trigger> .5 && gamepad1.left_trigger>.5){

                robot.lift.setTargetPosition(-70);
                robot.lift.setPower(1);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
//liftEND
//TrackingCode:
            LLStatus status = limelight.getStatus();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addLine("robot is sentient");
                // Access general information
                Pose3D botpose = result.getBotpose();
                double distance = getdistance(result.getTa());
                double tx = result.getTx();
                double txnc = result.getTxNC();
                double ty = result.getTy();
                double tync = result.getTyNC();


                //shootCODE



                boolean midSpeed = gamepad1.dpad_up;
                if (midSpeed){
                    targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
                    setpointRPM = (415.2 * Math.log(distance)) + 1173.8+25;
                    // flywheelon = true;
                } else if (gamepad1.dpad_down) {
                    targetTicksPerSec = 0;
                }

                // telemetry.addData("balls are in?", filled);
//                if(gamepad1.b && Math.abs(measuredRPM - setpointRPM) <= 50){
//                    robot.shootServo.setPosition(0);
//                    robot.intake.setVelocity(TICKS_PER_REV_INTAKE*1100/60);
//                    if(allowUp) {
//                        telemetry.addData("Everson","is the goat 3 ");
//                        robot.lift.setTargetPosition(-55);
//                        robot.lift.setPower(1);
//                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    }
//                }else{
//                    robot.lift.setTargetPosition(1);
//                    robot.lift.setPower(-1);
//                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }


                //shootEND

//                telemetry.addData("distance", distance);
//                telemetry.addData("tx", result.getTx());
////                telemetry.addData("txnc", result.getTxNC());
////                telemetry.addData("ty", result.getTy());
////                telemetry.addData("tync", result.getTyNC());
//                telemetry.addData("target area", result.getTa());
//
//                telemetry.addData("Botpose", botpose.toString());
                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int apriltagID = fr.getFiducialId();

//
//                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    if(apriltagID == 20) {

                        double targetX = fr.getTargetXDegrees();
                        double turretpidOutput = turretpid.calculate(0, targetX);

                        double turretfeedforward = 0;
                        double AngleError = -tx;
                        telemetry.addData("distance",distance);


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

//                        telemetry.addData("Limelight Target X", targetX);
//                        telemetry.addData("Final Power", motorPower);
//                        telemetry.addData("Angle", turretDegrees);
                    }
                }
            } else if (gamepad1.y) {
                int encoderTicks = robot.turret.getCurrentPosition();
                double currentDegrees = (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;

                if (currentDegrees > 5) {
                    robot.turret.setPower(-0.3);
                }
                else if (currentDegrees < -5) {
                    robot.turret.setPower(0.3);
                }
                else {
                    robot.turret.setPower(0);
                }

//                telemetry.addData("Mode", "Manual Reset");
//                telemetry.addData("Angle", currentDegrees);
            }
            else {
                // telemetry.addData("Limelight", "No data available");
                robot.turret.setPower(0);
            }
//TrackingCodeEND


            telemetry.update();
updateIntakeServo();
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
        //  double targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
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