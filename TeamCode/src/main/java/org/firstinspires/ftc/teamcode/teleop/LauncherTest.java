//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Projects.HWMap;
//
//@TeleOp(name = "LauncherTest")
//
//public class LauncherTest extends LinearOpMode {
//    public HWMap robot = new HWMap();
//    public ElapsedTime buttonTimer = new ElapsedTime();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//
//        double speed = 1;
//        waitForStart();
//        boolean isSpinning = false;
//        int ticksPerRev = 28;
//        double targetRPM = 0;
//        double rpmStep = 125; // 500 RPM per tap
//        double maxRPM = 6000; // motor max free speed
//        double minRPM = 0;    // optional lower bound
//
//        boolean lastUp = false;
//        boolean lastDown = false;
//        while (opModeIsActive()) {
//            boolean aButtonHeld = false;
//            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = -gamepad1.right_stick_x;
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio, but only when
//            // at least one is out of the range [-1, 1]
////            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////            double frontLeftPower = -(y + x + rx) / denominator;
////            double backLeftPower = -(y - x + rx) / denominator;
////            double frontRightPower = -(y - x - rx) / denominator;
////            double backRightPower = -(y + x - rx) / denominator;
////
////            robot.fLeftWheel.setPower(frontLeftPower * speed);
////            robot.bLeftWheel.setPower(backLeftPower * speed);
////            robot.fRightWheel.setPower(frontRightPower * speed);
////            robot.bRightWheel.setPower(backRightPower * speed);
//            robot.launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            boolean upPressed = gamepad1.dpad_up;
//            boolean downPressed = gamepad1.dpad_down;
//
//            if (upPressed && !lastUp) {
//                targetRPM += rpmStep;
//                targetRPM = Math.min(targetRPM, maxRPM);
//                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }
//
//            if (downPressed && !lastDown) {
//                targetRPM -= rpmStep;
//                targetRPM = Math.max(targetRPM, minRPM);
//                robot.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }
//
//            if (gamepad1.a){
//                targetRPM = -3;
//            }
//
//            lastUp = upPressed;
//            lastDown = downPressed;
//
//            double targetTicksPerSec = targetRPM / 60.0 * ticksPerRev;
//
//            robot.launcher.setVelocity(targetTicksPerSec);
//            double currentRPM = robot.launcher.getVelocity() / ticksPerRev * 60.0;
//
//            telemetry.addData("TargetRPM", targetRPM);
//            telemetry.addData("CurrentRPM", currentRPM);
//            telemetry.update();
//        }
//    }
//}