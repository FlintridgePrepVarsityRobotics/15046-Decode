
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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Projects.newHWmap;

import java.util.List;



@Config
@TeleOp(name = "blue Reg teleop")
public class regionalBlueTele extends LinearOpMode {
    public static double TP = 0.01;
    public static double TI = 0.00015;
    public static double TD = 0.00000005;

    public static double kP = 0.006;

    public static double kI = 0.2;
    public static double kD = 0.00026;
    public static double kF = 0.00042;

    // Feedforward: kS (static), kV (velocity), kA (acceleration)
    // kV roughly ~ 1 / (max_ticks_per_sec) as a starting point

    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.2;

    PIDController turretpid = new PIDController(TP, TI, TD);

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
    final double TICKS_PER_REV_INTAKE = 101.08;

    double targetTicksPerSec = 0;
    final double PROX_DIhST1 = 7.5;
    final double PROX_DIhST2 = 6.0;
    final double PROX_DIhST3 = 5.5;


    @Override
    public void runOpMode() throws InterruptedException {

//variables:
        boolean centered = false;
        boolean lastAState = false;
        boolean intakeFull = false;
        boolean isIntakeRunning = false;
        boolean allowUp = true;
        boolean color1 = false;
        boolean color2 = false;
        boolean color3 = false;

        boolean sense1 = false;
        boolean sense2 = false;
        boolean sense3 = false;
        double setpointRPM = 0;
        boolean flywheelon = false;
        int ticksPerRev = 28;

        robot.init(hardwareMap);


//setting modes, information on turret, limelight, telemetry
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */

//        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.setMsTransmissionInterval(5);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

//DriveCode:
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = gamepad1.right_stick_x;
            double speed = 1;

            double measuredTicksPerSec = robot.flywheel.getVelocity();
            double measuredRPM = measuredTicksPerSec / ticksPerRev * 60.0;


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

            if (gamepad1.right_bumper) {
                allowUp = true;

            }
            if (gamepad1.left_bumper) {
                allowUp = false;
            }
            if (gamepad1.right_trigger > .5 && gamepad1.left_trigger > .5) {


                robot.lift.setTargetPosition(-100);
                robot.lift.setPower(0.75);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            //end lift code

            //intake
            if (gamepad1.x) {
                robot.intake.setVelocity(0);
            }
            if (gamepad1.a && !intakeFull && buttonTimer.seconds() > 0.5) {
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 950 / 60);
                    robot.shootServo.setPosition(0.5);
                    buttonTimer.reset();
                }
                buttonTimer.reset();
            }

// 4. AUTO-STOP LOGIC
            if (intakeFull && isIntakeRunning) {
                isIntakeRunning = false;
                robot.intake.setVelocity(0);
            }

// Priority 1: SHOOTING (Button B)
            boolean isShooting = gamepad1.b && (Math.abs(measuredRPM - setpointRPM) <= 50);

            if (isShooting) {
                robot.shootServo.setPosition(0);
                robot.intake.setVelocity(TICKS_PER_REV_INTAKE * 1450 / 60);
                telemetry.addData("Everson", "is the goat 1");
                if (allowUp) {
                    robot.lift.setTargetPosition(-55);
                    robot.lift.setPower(1);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    robot.lift.setTargetPosition(0);
                    robot.lift.setPower(-1);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addLine("brakepad retract Everson is goat");
                }
            }

            if (!isIntakeRunning && !gamepad1.b) {
                if(gamepad1.right_trigger < .5 && gamepad1.left_trigger < .5) {
                    robot.intake.setVelocity(0);
                    robot.shootServo.setPosition(.5);
                    robot.lift.setTargetPosition(0);
                    robot.lift.setPower(-1);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addLine("brakepad retract Everson is goat2");
                }
            }


//FlywheelCode:
            pidf.setPIDF(kP, kI, kD, kF);
            feedforward = new SimpleMotorFeedforward(kS, kV, kA);

            double ffOutput = feedforward.calculate(targetTicksPerSec);

            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            double combinedOutput = ffOutput + pidOutput;
            combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            robot.flywheel.setPower(combinedOutput);
//LauncherCodeEND

// --- SENSOR READING ---
            // ----------------------------------------------------------------------
            double dist1 = ((ColorRangeSensor) robot.sensor1).getDistance(DistanceUnit.CM);
            double dist2 = ((ColorRangeSensor) robot.sensor2).getDistance(DistanceUnit.CM);
            double dist3 = ((ColorRangeSensor) robot.sensor3).getDistance(DistanceUnit.CM);

            sense1 = dist1 < 7.5;
            sense2 = dist2 < 6.3;
            sense3 = dist3 < 6;

            telemetry.addData("dihstances (cm)", "1: %.1f, 2: %.1f, 3: %.1f", dist1, dist2, dist3);
            telemetry.addData("Dihtected?", "1: %b, 2: %b, 3: %b", sense1, sense2, sense3);

            if (sense1 && sense2 && sense3) {
                if (colorTimer.seconds() > 0.3) {
                    intakeFull = true;
                    telemetry.addData("Status", "intakefull");
                }
            } else {
                colorTimer.reset();
                intakeFull = false;
            }
//liftEND
//TrackingCode:
            if (gamepad1.dpad_down) {
                targetTicksPerSec = 0;
                robot.flywheel.setPower(0);
            }
            LLStatus status = limelight.getStatus();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                // Access general information
                Pose3D botpose = result.getBotpose();
                double distance = getDistance(result.getTa());
                double tx = result.getTx();
                double txnc = result.getTxNC();
                double ty = result.getTy();
                double tync = result.getTyNC();
                telemetry.addLine("eversonisgoat robot sees apritag");

                //shootCODE
                boolean midSpeed = gamepad1.dpad_up;
                if (midSpeed) {
                    targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
                    setpointRPM = (415.2 * Math.log(distance)) + 1173.8 + 25;
                    // flywheelon = true;
                }

                // telemetry.addData("balls are in?", filled);
                //shootEND
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int apriltagID = fr.getFiducialId();
                    if (apriltagID == 20) {

                        telemetry.addLine("eversonisgoat robot sees correct apritag");

                        double targetX = fr.getTargetXDegrees();
                        double turretpidOutput = turretpid.calculate(0, targetX);

                        double turretfeedforward = 0;
                        double AngleError = -targetX;

                        if (Math.abs(turretpidOutput) > 0.01) {
                            turretfeedforward = Math.signum(turretpidOutput) * MIN_POWER_TO_MOVE;
                        }

                        double dynamicTolerance = 100.0 / distance; //use distance and if not accurate enough, decrease 100.0 justin everson isaac
                        dynamicTolerance = Range.clip(dynamicTolerance, .5, 4);

                        telemetry.addData("Dynamic Tolerance", dynamicTolerance);

                        double motorPower;

                        if (Math.abs(AngleError) < dynamicTolerance)
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

                    }
                }
            }
            else if (gamepad1.y) {
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

            }
            else {
                // telemetry.addData("Limelight", "No data available");
                robot.turret.setPower(0);
            }
//TrackingCodeEND

            telemetry.update();

        }
//        limelight.stop();
    }
    public double getDistance (double ta){
        double scale = 10;
        double newDistance = scale / ta;
        return (newDistance);
    }
    public double getDistanceangle(double ty) {
        double limelightMountAngle = 25.0;
        double limelightHeightInches = 10.0;
        double goalHeightInches = 29.5;
        double angleToGoal = limelightMountAngle + ty;
        double angleRadians = angleToGoal * (Math.PI / 180.0);

        return (goalHeightInches - limelightHeightInches) / Math.tan(angleRadians);
    }
}
