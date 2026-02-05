package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class HWMapOld {
    public DcMotor fLeftWheel = null;  //expansion hub port0
    public DcMotor fRightWheel = null; //control hub port0
    public DcMotor bLeftWheel = null; //expansion hub port1
    public DcMotor bRightWheel = null; //control hub port1

    public Limelight3A limelight = null;
    public ColorSensor sensor1 = null;
    public ColorSensor sensor2 = null;
//    public DcMotor outtake = null;
    public DcMotorEx intake = null;
    public WebcamName camera = null; //usb 3 port

    public CRServo intakeServo = null;
    public DcMotorEx launcher = null; //control hub 2
    public void init(HardwareMap hwMap) {
        fLeftWheel = hwMap.dcMotor.get("FL");
        fRightWheel = hwMap.dcMotor.get("FR");
        bLeftWheel = hwMap.dcMotor.get("BL");
        bRightWheel = hwMap.dcMotor.get("BR");
        sensor1 = hwMap.colorSensor.get("sensor1");
        sensor2 = hwMap.colorSensor.get("sensor2");
        intake = hwMap.get(DcMotorEx.class, "intake");
        intakeServo = hwMap.crservo.get("intakeServo");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        limelight = hwMap.get(Limelight3A.class, "limelight");
//        leftencoder = hwMap.dcMotor.get("leftencoder");
//        rightencoder = hwMap.dcMotor.get("rightencoder");
//        backencoder = hwMap.dcMotor.get("backencoder");
        camera = hwMap.get(WebcamName.class, "webcam");
//
//        // Get motors from hardware map
        fRightWheel.setDirection(DcMotor.Direction.REVERSE);
        fLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        bRightWheel.setDirection(DcMotor.Direction.REVERSE);
        bLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);


//
//        //set direction
        fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//<<<<<<< HEAD
//
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        camera = hwMap.get(WebcamName.class, "webcam");

        // Get motors from hardware map
//
//        fRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
