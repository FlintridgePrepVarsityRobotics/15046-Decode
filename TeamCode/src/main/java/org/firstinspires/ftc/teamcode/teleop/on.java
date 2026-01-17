
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
import org.firstinspires.ftc.teamcode.Projects.motortestHWmap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Config
@TeleOp(name = "on")
public class on extends LinearOpMode {

    public motortestHWmap robot = new motortestHWmap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .7;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Motor speed", speed);
            telemetry.update();
            if (gamepad1.dpad_right) {
                speed = .5;

                sleep(200);
            }
            if (gamepad1.dpad_up) {
                speed += .05;

                sleep(200);
            }
            if (gamepad1.dpad_down) {
                speed -= .05;

                sleep(200);
            }
            if (gamepad1.b) {
                speed =0;
robot.spin.setPower(speed);
                sleep(200);
            }

            if (gamepad1.a) {
                robot.spin.setPower(speed);
            }
        }
    }
}
