package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HWMap {
    public DcMotor fLeftWheel = null;  //control hub port 2
    public DcMotor fRightWheel = null; //control hub port 3
    public DcMotor bLeftWheel = null; //control hub port 1
    public DcMotor bRightWheel = null; //control hub port 0
    public Servo testServo = null;
   public WebcamName camera = null;

    public void init(HardwareMap hwMap) {
        fLeftWheel = hwMap.dcMotor.get("FL");
        fRightWheel = hwMap.dcMotor.get("FR");
        bLeftWheel = hwMap.dcMotor.get("BL");
        bRightWheel = hwMap.dcMotor.get("BR");
        testServo = hwMap.servo.get("testServo"); // on  driver hub, input the name within the quotations

       camera = hwMap.get(WebcamName.class, "webcam");

        // Get motors from hardware map
        fRightWheel.setDirection(DcMotor.Direction.REVERSE);
        fLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        bRightWheel.setDirection(DcMotor.Direction.REVERSE);
        bLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        //set direction
        fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Stop();
    }

    public void Stop() {
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        testServo.setPosition(1);
    }
}
