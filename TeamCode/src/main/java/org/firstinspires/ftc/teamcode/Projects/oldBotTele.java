package org.firstinspires.ftc.teamcode.Projects;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "old tele FIXED")
public class oldBotTele extends LinearOpMode {

    // PID values (tune these)
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.00025;
    public static double kF = 0.0;   // NOT used directly in FTC power control

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public HWMapOld robot = new HWMapOld();

    double targetTicksPerSec = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // 🔴 REQUIRED: initialize hardware
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            /* =========================
               DRIVE CODE (Mecanum)
               ========================= */
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x ;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower);
            robot.bLeftWheel.setPower(backLeftPower);
            robot.fRightWheel.setPower(frontRightPower);
            robot.bRightWheel.setPower(backRightPower);

            /* =========================
               INTAKE (HOLD A)
               ========================= */
            if (gamepad1.a) {
                robot.intake.setPower(1.0);

                // CHANGE THIS depending on servo type
                // If CRServo:
                robot.intakeServo.setPower(1.0);

                // If NORMAL servo instead, comment above and use:
                // robot.intakeServo.setPosition(1.0);
            } else {
                robot.intake.setPower(0.0);

                // CRServo:
                robot.intakeServo.setPower(0.0);

                // Normal servo:
                // robot.intakeServo.setPosition(0.5);
            }

            /* =========================
               LAUNCHER PID VELOCITY
               ========================= */

            // Hold dpad_up to spin launcher
            if (gamepad1.dpad_up) {
                targetTicksPerSec = 1300;
            } else {
                targetTicksPerSec = 0;
            }

            double measuredTicksPerSec = robot.launcher.getVelocity();

            double pidOutput = pidf.calculate(measuredTicksPerSec, targetTicksPerSec);

            // Clamp to motor power range
            pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

            robot.launcher.setPower(pidOutput);

            /* =========================
               TELEMETRY
               ========================= */
            telemetry.addData("Target TPS", targetTicksPerSec);
            telemetry.addData("Measured TPS", measuredTicksPerSec);
            telemetry.addData("Launcher Power", pidOutput);
            telemetry.update();
        }
    }
}