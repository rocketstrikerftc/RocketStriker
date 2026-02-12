package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TestBench {

    private IMU imu;

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
    }

    public YawPitchRollAngles getOrientation() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles();
        } else {
            return new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0); // evita null
        }
    }
}
