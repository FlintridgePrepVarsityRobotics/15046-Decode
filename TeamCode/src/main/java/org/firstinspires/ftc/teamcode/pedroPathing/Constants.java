package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(9.8)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.009, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.07,0.0,0.01,0.6,0.0))//not finished
    .forwardZeroPowerAcceleration(-46.596194)
    .lateralZeroPowerAcceleration(-73.617);

//        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.007, .03))
//        .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.06, 0.02))
//        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.00001,0.3,0.01))
//        .centripetalScaling(0.003);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .7, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//60.0469213
            .xVelocity(60.0952777777)
            .yVelocity(69.248)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .turnTicksToInches(.002565)
            .forwardTicksToInches(.0021106)
            .strafeTicksToInches(.0030509)
            .leftPodY(4)
            .rightPodY(-4)
            .strafePodX(-8)
            .leftEncoder_HardwareMapName("FL")
            .rightEncoder_HardwareMapName("FR")
            .strafeEncoder_HardwareMapName("BR")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE);
}
//Everson was here