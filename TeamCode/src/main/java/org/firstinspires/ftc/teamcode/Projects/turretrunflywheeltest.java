package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "flywheel_turret_tracker")
public class turretrunflywheeltest extends LinearOpMode {
    public turretHWMap robot = new turretHWMap();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.update();

        waitForStart();

        double speed = 0.5;

        while (opModeIsActive()){
            if (gamepad1.dpad_down) {
                speed -= 0.05;
                sleep(100);
            }
            if (gamepad1.dpad_up) {
                speed += 0.05;
                sleep(100);
            }

            if (gamepad1.a) {
                robot.flywheel.setPower(speed);
            } else {
                robot.flywheel.setPower(0);
            }

            telemetry.addData("Flywheel Target Speed", "%.2f", speed);
            telemetry.update();
        }
    }
}