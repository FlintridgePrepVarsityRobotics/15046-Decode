package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//kjnlkjn

public class HWMap {
    public DcMotor fLeftWheel = null;  //expansion hub port0
    public DcMotor fRightWheel = null; //control hub port0
    public DcMotor bLeftWheel = null; //expansion hub port1
    public DcMotor bRightWheel = null; //control hub port1
//    public DcMotor outtake = null;
    public DcMotor intake = null;
    public WebcamName camera = null; //usb 3 port
//    public DcMotor leftencoder = null;// ch port 0
//    public DcMotor rightencoder = null;// ch port 1
//    public DcMotor backencoder = null;//ch port 2
    public CRServo intakeServo = null;
    public DcMotorEx launcher = null; //control hub 2
    public void init(HardwareMap hwMap) {
        fLeftWheel = hwMap.dcMotor.get("FL");
        fRightWheel = hwMap.dcMotor.get("FR");
        bLeftWheel = hwMap.dcMotor.get("BL");
        bRightWheel = hwMap.dcMotor.get("BR");
        intake = hwMap.dcMotor.get("intake");
        intakeServo = hwMap.crservo.get("intakeServo");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
//        leftencoder = hwMap.dcMotor.get("leftencoder");
//        rightencoder = hwMap.dcMotor.get("rightencoder");
//        backencoder = hwMap.dcMotor.get("backencoder");
        camera = hwMap.get(WebcamName.class, "webcam");
//
//        // Get motors from hardware map
        fRightWheel.setDirection(DcMotor.Direction.FORWARD);
        fLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        bRightWheel.setDirection(DcMotor.Direction.FORWARD);
        bLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

//        leftencoder.setDirection(DcMotor.Direction.FORWARD);
//        rightencoder.setDirection(DcMotor.Direction.FORWARD);
//        backencoder.setDirection(DcMotor.Direction.FORWARD);
//
//        //set direction
        fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//<<<<<<< HEAD
//        leftencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
//        leftencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        camera = hwMap.get(WebcamName.class, "webcam");

        // Get motors from hardware map
//
        fRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Stop();


    }

    public void Stop() {
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        intake.setPower(0);
        intakeServo.setPower(0);
        launcher.setPower(0);
    }

}

//albert's test commit and push1234
//Everson was here
