package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@Disabled
@Autonomous(name = "odometria ria")
public class odometria extends LinearOpMode {

    DcMotor FL0;
    DcMotor FR1; // POD eixo Y
    DcMotor BL2; // POD eixo X
    DcMotor BR3;

    IMU imu;

    static final int TOLERANCIA_X = 50; // ticks aceitáveis
    static final int TARGET_Y = 4000;

    @Override
    public void runOpMode() {

        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        FL0.setDirection(DcMotor.Direction.FORWARD);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BR3.setDirection(DcMotor.Direction.REVERSE);

        FL0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Pronto - Odometria X/Y (correção no final)");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        int yInicial = FR1.getCurrentPosition();
        int xInicial = BL2.getCurrentPosition();

        // =============================
        // 1️⃣ ANDAR PARA TRÁS (EIXO Y)
        // =============================
        while (opModeIsActive() &&
                Math.abs(FR1.getCurrentPosition() - yInicial) < TARGET_Y) {

            FL0.setPower(-0.8);
            FR1.setPower(-0.8);
            BL2.setPower(-0.8);
            BR3.setPower(-0.8);

            telemetry.addData("Y", FR1.getCurrentPosition() - yInicial);
            telemetry.addData("X", BL2.getCurrentPosition() - xInicial);
            telemetry.update();
        }

        stopMotors();
        sleep(200);

        // ==================== =========
        // 2️⃣ CORRIGIR X (SÓ NO FINAL)
        // =============================
        int erroX = BL2.getCurrentPosition() - xInicial;

        while (opModeIsActive() && Math.abs(erroX) > TOLERANCIA_X) {

            double direcao = erroX > 0 ? -0.4 : 0.4;

            // STRAFE
            FL0.setPower(direcao);
            FR1.setPower(-direcao);
            BL2.setPower(-direcao);
            BR3.setPower(direcao);

            erroX = BL2.getCurrentPosition() - xInicial;

            telemetry.addData("CORRIGINDO X", erroX);
            telemetry.update();
        }

        stopMotors();
        telemetry.addLine("Autônomo finalizado");
        telemetry.update();
    }

    private void stopMotors() {
        FL0.setPower(0);
        FR1.setPower(0);
        BL2.setPower(0);
        BR3.setPower(0);
    }
}