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

        float hsv1[] = {0F, 0F, 0F};
        float hsv2[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        waitForStart();

        while (opModeIsActive()) {

            // Convert the RGB values to HSV
            Color.RGBToHSV(
                    (int) (robot.sensor1.red() * SCALE_FACTOR),
                    (int) (robot.sensor1.green() * SCALE_FACTOR),
                    (int) (robot.sensor1.blue() * SCALE_FACTOR),
                    hsv1
            );

            Color.RGBToHSV(
                    (int) (robot.sensor2.red() * SCALE_FACTOR),
                    (int) (robot.sensor2.green() * SCALE_FACTOR),
                    (int) (robot.sensor2.blue() * SCALE_FACTOR),
                    hsv2
            );

            telemetry.addData("Red", robot.sensor1.red());
            telemetry.addData("Green", robot.sensor1.green());
            telemetry.addData("Blue", robot.sensor1.blue());
            telemetry.addData("Hue1", hsv1[0]);
            telemetry.addData("Hue2", hsv2[0]);
            telemetry.update();
            float hue1 = hsv1[0];
            float hue2 = hsv2[0];
            if(hue1 < 30){

                telemetry.addData("Color1", "Red");

            }

            else if (hue1 < 60) {

                telemetry.addData("Color1", "Orange");

            }

            else if (hue1 < 140){

                telemetry.addData("Color1", "Yellow");

            }

            else if (hue1 < 180){ //green --> 160

                telemetry.addData("Color1", "Green");

            }

            else if (hue1 < 220){

                telemetry.addData("Color1", "Blue");

            }

            else if (hue1 < 270){ //purple --> 230-250

                telemetry.addData("Color1", "Purple");

            }

            else{

                telemetry.addData("Color1", "Red");

            }

            if(hue2 < 30){

                telemetry.addData("Color2", "Red");

            }

            else if (hue2 < 60) {

                telemetry.addData("Color2", "Orange");

            }

            else if (hue2 < 140){

                telemetry.addData("Color2", "Yellow");

            }

            else if (hue2 < 180){ //green --> 160

                telemetry.addData("Color2", "Green");

            }

            else if (hue2 < 220){

                telemetry.addData("Color2", "Blue");

            }

            else if (hue2 < 270){ //purple --> 230-250

                telemetry.addData("Color2", "Purple");

            }

            else{

                telemetry.addData("Color2", "Red");

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
