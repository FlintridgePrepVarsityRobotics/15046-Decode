//package org.firstinspires.ftc.teamcode.teleop;

package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Projects.coloorsensHWmap;

@Config
@TeleOp(name = "ColourSensor")
public class COLOURSENSOUR extends LinearOpMode {

    public coloorsensHWmap robot = new coloorsensHWmap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        waitForStart();

        while (opModeIsActive()) {

            // Convert the RGB values to HSV
            Color.RGBToHSV(
                    (int) (robot.sensor1.red() * SCALE_FACTOR),
                    (int) (robot.sensor1.green() * SCALE_FACTOR),
                    (int) (robot.sensor1.blue() * SCALE_FACTOR),
                    hsvValues
            );

            telemetry.addData("Red", robot.sensor1.red());
            telemetry.addData("Green", robot.sensor1.green());
            telemetry.addData("Blue", robot.sensor1.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            float hue = hsvValues[0];
            if(hue < 30){

                telemetry.addData("Color", "Red");

            }

            else if (hue < 60) {

                telemetry.addData("Color", "Orange");

            }

            else if (hue < 140){

                telemetry.addData("Color", "Yellow");

            }

            else if (hue < 180){ //green --> 160

                telemetry.addData("Color", "Green");

            }

            else if (hue < 220){

                telemetry.addData("Color", "Blue");

            }

            else if (hue < 270){ //purple --> 230-250

                telemetry.addData("Color", "Purple");

            }

            else{

                telemetry.addData("Color", "Red");

            }

            telemetry.update();

        }






    }
    }


//
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.dashboard.config.Config;
//
//import android.graphics.Color;
//import android.util.Size;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Projects.coloorsensHWmap;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//@Config
//@TeleOp(name = "ColourSensor")
//public class COLOURSENSOUR extends LinearOpMode {
//
//    public coloorsensHWmap robot = new coloorsensHWmap();
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//
//
//
//        // For A-button toggle
//        boolean lastAState = false;
//
//
//
//
//        // hsvValues is an array that will hold the hue, saturation, and value information.
//        float hsvValues[] = {0F, 0F, 0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
//        // sometimes it helps to multiply the raw RGB values with a scale factor
//        // to amplify/attentuate the measured values.
//        final double SCALE_FACTOR = 255;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//
//            telemetry.addData("Clear", robot.sensor1.red());
//            telemetry.addData("Red  ", robot.sensor1.green());
//            telemetry.addData("Blue ", robot.sensor1.blue());
//            telemetry.addData("Hue", hsvValues[0]);
//            telemetry.update();
//
//
//            // --- AprilTag Centering (Y button) ---
//
//        }
//    }
//}
