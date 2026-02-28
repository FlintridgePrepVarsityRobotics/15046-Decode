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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(28.56)//added 1 1/2 pounds to robot mass after Aurix said [redacted]
//            .forwardZeroPowerAcceleration(-38.7525387791384267)
//            .lateralZeroPowerAcceleration(-76.6048014843286867)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, .026))
//            .headingPIDFCoefficients(new PIDFCoefficients(.8, 0, 0.005, 0.015))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.025, .000001, .00001, .6,.01))
//            .centripetalScaling(0.0005)
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)

                .pathConstraints(pathConstraints)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//60.0469213
//            .xVelocity(72.6471077460621867)
//            .yVelocity(55.4962444965)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.00294564308)
            .strafeTicksToInches(0.00295376722397)
            .turnTicksToInches(0.00297964376732)
            .leftPodY((8.625)/2)
            .rightPodY(-(8.625)/2)
            .strafePodX(-6.86325)
            .leftEncoder_HardwareMapName("FL")
            .rightEncoder_HardwareMapName("FR")
            .strafeEncoder_HardwareMapName("BR")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

}
//Everson was here