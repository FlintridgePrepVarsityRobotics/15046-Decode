package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Projects.newHWmap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import java.util.List;


@Config
@TeleOp(name = "red Reg teleop")
public class regionalRedTele extends LinearOpMode {
    public static double IP = 0.0005;
    public static double II = 0;
    public static double ID = 0;
    public static double IF = 0.000048;
    public static double IS = 0.0;
    public static double IV = 0.0;
    public static double IA = 0.2;
    PIDFController Intakepidf = new PIDFController(IP, II, ID, IF);
    SimpleMotorFeedforward feedforwardIntake = new SimpleMotorFeedforward(IS, IV, IA);
    public static double TP = 0.013;
    public static double TI = 0.0001;
    public static double TD = 0.00000005;


    public static double kP = 0.006;
    public static double kI = 0.04;
    public static double kD = 0.00008;
    public static double kF = 0.00042;


    //blue
//    public static double goalX = 0.0;
//    public static double goalY = 144.0;
//    public static double tagX = 48.0;
//    public static double tagY = 130.0;
//    public static double resetX = 22.0;
//    public static double resetY = 120.0;
//    public static double resetHeading = 140.0; //107.5


    //red
    public static double goalX = 144.0;
    public static double goalY = 144.0;
    public static double tagX = 96.0;
    public static double tagY = 130.0;
    public static double resetX = 122.0;
    public static double resetY = 120.0;
    public static double resetHeading = 36.5; //36.5
    private Follower follower;


    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.2;


    public static double tagOffset = -8.0; //negative for red


    PIDController turretpid = new PIDController(TP, TI, TD);
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);


    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime colorTimer = new ElapsedTime();


    private Limelight3A limelight;
    public newHWmap robot = new newHWmap();


    final double TICKS_PER_REV = 294.0;      // GoBilda 5202/5203
    final double GEAR_RATIO = 0.3953;        // 34 / 86
    final double MAX_DEGREES = 70;
    double ticksPerRevIntake = 101.08;


    double targetTicksPerSec = 0;
    double setpointRPMIntake = 0;


    @Override
    public void runOpMode() throws InterruptedException {


//variables:
        boolean intakeFull = false;
        boolean isIntakeRunning = false;
        boolean isIntakeReversed = false;
        boolean allowUp = true;


        boolean sense1 = false;
        boolean sense2 = false;
        boolean sense3 = false;
        double setpointRPM = 0;


        boolean flywheelon = false;
        int ticksPerRev = 28;
        boolean trackingAllowed = false;
        boolean FirstYPress = true;
        boolean isTurretLocked = false;
        double lockedTurretTarget = 0.0;


        robot.init(hardwareMap);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(resetX, resetY, Math.toRadians(resetHeading)));


//setting modes, info on turret, limelight, telemetry
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot.fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        telemetry.setMsTransmissionInterval(5);
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            follower.update();
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
                robot.lift.setPower(1);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //end lift code


            //intake
            if (gamepad1.x && buttonTimer.seconds() > 0.5) {
                isIntakeReversed = !isIntakeReversed;
                if (isIntakeReversed) {
                    isIntakeRunning = false;
                    robot.intake.setPower(-0.6);
                } else {
                    robot.intake.setPower(0);
                }
                buttonTimer.reset();
            }


            if (gamepad1.a && !intakeFull && buttonTimer.seconds() > 0.5) {
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    isIntakeReversed = false;
                    setpointRPMIntake = 700;
                    robot.shootServo.setPosition(0.5);
                    runIntake();
                }
                buttonTimer.reset();
            }


//auto-stop
            Intakepidf.setPIDF(IP, II, ID, IF);
            feedforwardIntake = new SimpleMotorFeedforward(IS, IV, IA);


            if (intakeFull && isIntakeRunning) {
                isIntakeRunning = false;
                setpointRPMIntake = 0;
                robot.intake.setVelocity(0);
            }


// shooting, btn b
            boolean isShooting = gamepad1.b && (Math.abs(measuredRPM - setpointRPM) <= 100);
            if (isShooting) {
                robot.shootServo.setPosition(0);
                setpointRPMIntake = 1900;
                runIntake();
                telemetry.addData("Everson", "is the goat 1");
                if (allowUp) {
                    robot.lift.setTargetPosition(-45);
                    robot.lift.setPower(1);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    robot.lift.setTargetPosition(0);
                    robot.lift.setPower(-1);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addLine("brakepad retract Everson is goat");
                }
            }
            if (!isShooting && gamepad1.b) {
                robot.shootServo.setPosition(0);
                setpointRPMIntake = 0;
                robot.intake.setVelocity(0);
            }


            if (!isIntakeRunning && !gamepad1.b) {
                if (gamepad1.right_trigger < .5 && gamepad1.left_trigger < .5) {
                    if (!isIntakeReversed) {
                        robot.intake.setVelocity(0);
                    }
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
                robot.flywheel.setPower(0);
                robot.flywheel.setVelocity(0);
                targetTicksPerSec = 0;
                setpointRPM = 0;
                isTurretLocked = false;
            }


            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();


            if (FirstYPress && gamepad1.y) {
                trackingAllowed = true;


                robot.turret.setPower(0);
                robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                follower.setStartingPose(new Pose(resetX, resetY, Math.toRadians(resetHeading)));
                telemetry.addData("e the goat", "reset turret, drive enc, and odo");
                if (result != null) {
                    telemetry.addData("Distance", getDistance(result.getTy()));
                }


                FirstYPress = false;
            }
            if (!FirstYPress && gamepad1.y) {
                robot.turret.setTargetPosition(0);
                sleep(500);


                robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                follower.setStartingPose(new Pose(resetX, resetY, Math.toRadians(resetHeading)));
                telemetry.addData("e the goat", "reset turret, drive enc, and odo");
                if (result != null) {
                    telemetry.addData("Distance", getDistance(result.getTy()));
                }
            }


            boolean onTag = false;
            double targetTurretDeg = 0;
            double dynamicTolerance = 2.0;
            double turretMotorPower = 0.0;


            int encoderTicks = robot.turret.getCurrentPosition();
            double currentTurretDeg = (encoderTicks / (TICKS_PER_REV / GEAR_RATIO)) * 360.0;


            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    // ID 24 is Red
                    if (fr.getFiducialId() == 24) {


                        double distance = getDistance(result.getTy());
                        double activeOffset = 0.0;
                        if (setpointRPM >= 2100) {
                            activeOffset = tagOffset;
                        }
                        double targetTx = Math.toDegrees(Math.atan(activeOffset / distance));


                        double tx = fr.getTargetXDegrees();
                        double calculatedTargetDeg = currentTurretDeg - (tx - targetTx);


                        dynamicTolerance = Range.clip(120.0 / distance, 0.3, 8.0);


                        boolean midSpeed = gamepad1.dpad_up;
                        if (midSpeed) {
                            targetTicksPerSec = setpointRPM / 60.0 * ticksPerRev;
                            if (distance < 70) {
                                setpointRPM = (494 * Math.log(distance)) - 105;
                            } else if (distance > 70) {
                                setpointRPM = (494 * Math.log(distance)) + 105;
                            }
                        }


                        if (setpointRPM < 2100) {
                            isTurretLocked = false;
                        }


                        if (!isTurretLocked) {
                            targetTurretDeg = calculatedTargetDeg;


                            if (setpointRPM >= 2100 && Math.abs(calculatedTargetDeg - currentTurretDeg) <= dynamicTolerance) {
                                isTurretLocked = true;
                                lockedTurretTarget = calculatedTargetDeg;
                            }
                        } else {
                            targetTurretDeg = lockedTurretTarget;
                            telemetry.addData("Turret Status", "loganLOCK");
                        }


                        onTag = true;
                        telemetry.addData("Turret Mode", "limelight");
                        telemetry.addData("tolerance", dynamicTolerance);
                        telemetry.addData("Setpoint RPM", setpointRPM);


                        TelemetryPacket packet = new TelemetryPacket();
                        FtcDashboard dashboard = FtcDashboard.getInstance();
                        dashboard.setTelemetryTransmissionInterval(25);
                        packet.put("Setpoint RPM", setpointRPM);
                        telemetry.addData("Measured RPM", "%.1f", measuredRPM);
                        packet.put("Measured RPM", measuredRPM);
                        dashboard.sendTelemetryPacket(packet);
                        telemetry.update();
                        break;
                    }
                }
            }
            if (!onTag && trackingAllowed) {
                double robotX = follower.getPose().getX();
                double robotY = follower.getPose().getY();
                double robotHeading = Math.toDegrees(follower.getPose().getHeading());


                double odoDistance = Math.hypot(goalX - robotX, goalY - robotY);


                if (setpointRPM <= 2100) {
                    double fieldAngleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
                    targetTurretDeg = normalizeDegrees(fieldAngleToGoal - robotHeading);
                    dynamicTolerance = 2.0;


                    onTag = true;
                    telemetry.addData("Turret Mode", "odo (pedro)");
                }
            }
            if (onTag) {
                double clampedTarget;
                if (Math.abs(targetTurretDeg) > 90.0) {
                    clampedTarget = 0.0;
                } else {
                    clampedTarget = Range.clip(targetTurretDeg, -MAX_DEGREES, MAX_DEGREES);
                }


                double angleError = Math.abs(clampedTarget - currentTurretDeg);


                turretMotorPower = turretpid.calculate(currentTurretDeg, clampedTarget);


                if (isTurretLocked) {
                    turretMotorPower = Range.clip(turretMotorPower, -0.08, 0.08);
                } else if (angleError <= dynamicTolerance) {
                    turretMotorPower = Range.clip(turretMotorPower, -0.12, 0.12);
                }
            } else {
                turretMotorPower = 0;
            }
            turretMotorPower = turretLims(turretMotorPower, currentTurretDeg);
            robot.turret.setPower(turretMotorPower);
//TrackingCodeEND
        }
    }


    // Angle-based distance (ty)
    public double getDistance(double ty) {
        double limelightMountAngle = 20.0;
        double limelightHeightInches = 14.0;
        double goalHeightInches = 29.5;
        double angleToGoal = limelightMountAngle + ty;
        double angleRadians = angleToGoal * (Math.PI / 180.0);
        return (goalHeightInches - limelightHeightInches) / Math.tan(angleRadians);
    }


    private double normalizeDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }


    private double turretLims(double power, double currentDegrees) {
        if (currentDegrees >= 70 && power > 0) return 0;
        if (currentDegrees <= -70 && power < 0) return 0;
        return power;
    }


    public void runIntake() {
        double targetTicksPerSecIntake = setpointRPMIntake / 60 * ticksPerRevIntake;
        double measuredTicksPerSecIntake = robot.intake.getVelocity();
        double ffOutputIntake = feedforwardIntake.calculate(targetTicksPerSecIntake);
        double pidOutputIntake = Intakepidf.calculate(measuredTicksPerSecIntake, targetTicksPerSecIntake);
        double combinedOutputIntake = ffOutputIntake + pidOutputIntake;
        combinedOutputIntake = Math.max(-1.0, Math.min(1.0, combinedOutputIntake));
        robot.intake.setPower(combinedOutputIntake);
    }
}

