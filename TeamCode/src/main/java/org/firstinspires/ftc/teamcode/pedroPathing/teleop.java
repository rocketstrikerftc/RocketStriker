package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Teleoperado")
public class teleop extends LinearOpMode {

    private Follower follower;

    public ElapsedTime runtime = new ElapsedTime();

    public Limelight3A limelight;
    public DcMotor FL0;
    public DcMotor FR1;
    public DcMotor BL2;
    public DcMotor BR3;
    public DcMotor SH4;
    public DcMotorEx VS5;
    public DcMotorEx HL6;
    public DcMotor LC7;




    double hl6Power = 0.0;
    double vs5Power = 0.0;
    boolean shutertLigado = false;

    boolean shuterAnterior = false;

    boolean lastDpadUp1 = false;

    boolean lastDpaDonw1 = false;
    boolean lastDpadUp2 = false;

    boolean lastDpaDonw2 = false;
    boolean lastDpadRight1 = false;
    boolean lastDpadRight2 = false;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");
        SH4 = hardwareMap.get(DcMotor.class, "SH4");
        VS5 = hardwareMap.get(DcMotorEx.class, "VS5");
        HL6 = hardwareMap.get(DcMotorEx.class, "HL6");
        LC7 = hardwareMap.get(DcMotor.class, "LC7");


        FL0.setDirection(DcMotor.Direction.FORWARD);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BR3.setDirection(DcMotor.Direction.REVERSE);
        SH4.setDirection(DcMotor.Direction.FORWARD);
        VS5.setDirection(DcMotor.Direction.REVERSE);
        HL6.setDirection(DcMotor.Direction.FORWARD);
        LC7.setDirection(DcMotor.Direction.FORWARD);



        follower = Constants.createFollower(hardwareMap);

        PIDFCoefficients pidf =
                new PIDFCoefficients(88.11, 0, 0, 13.21);
        PIDFCoefficients pidf1 =
                new PIDFCoefficients(114.0040, 0, 0, 13.4010);

        VS5.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,pidf);
        HL6.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,pidf1);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            // Informações gerais
            Pose3D botpose = result.getBotpose();
            telemetry.addData("Botpose", botpose.toString());

            // --- Detecção de AprilTags (estilo AprilTagDetection) ---
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials == null || fiducials.isEmpty()) {
                telemetry.addLine("Nenhuma AprilTag detectada.");
            } else {
                telemetry.addLine("Tags detectadas:");

                for (LLResultTypes.FiducialResult fid : fiducials) {
                    int tagId = fid.getFiducialId();
                    String family = fid.getFamily();
                    double x = fid.getTargetXDegrees();
                    double y = fid.getTargetYDegrees();

                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("Family", family);
                    telemetry.addData("Position", String.format("X: %.2f°, Y: %.2f°", x, y));

                    // Ações diferentes por ID
                    switch (tagId) {
                        case 20:
                            telemetry.addLine("→ AÇÃO para tag 21");
                            break;
                    }






        telemetry.update();

        waitForStart();
        runtime.reset();
        follower.update();


        while (opModeIsActive()) {

            follower.update();






            double axial = -gamepad1.left_stick_y;

            double lateral = 0;
            if (gamepad1.left_bumper)  lateral = -1;
            if (gamepad1.right_bumper) lateral =  1;

            double yaw = gamepad1.right_stick_x;


            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double max = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            FL0.setPower(leftFrontPower);
            FR1.setPower(rightFrontPower);
            BL2.setPower(leftBackPower);
            BR3.setPower(rightBackPower);

            if (gamepad1.a) {
                SH4.setPower(1);
            } else {
                SH4.setPower(0);
            }

            if (gamepad2.right_bumper){
                SH4.setPower(1);
                LC7.setPower(1);
            } else {
                SH4.setPower(0);
                LC7.setPower(0);
            }

            if (gamepad2.left_bumper){
                SH4.setPower(1);

            } else {
                SH4.setPower(0);
                
            }

            if (gamepad2.x){
                SH4.setPower(-1);
                LC7.setPower(-1);
            } else {
                SH4.setPower(0);
                LC7.setPower(0);
            }
            boolean shuterAtual= gamepad2.b;
            // Detecta o clique (quando o botão é pressionado)
            if (shuterAtual && !shuterAnterior) {
                shutertLigado = !shutertLigado; // alterna liga/desliga
            }
            // Atualiza o estado anterior do botão
            shuterAnterior = shuterAtual;




           if (gamepad2.aWasPressed()){
               vs5Power = 0.20;
               hl6Power = 0.50;
           }

            if (gamepad2.yWasPressed()){
                vs5Power = 0.40;
                hl6Power = 0.60;
            }


            if (shutertLigado){
                VS5.setPower(vs5Power);
                HL6.setPower(hl6Power);
            } else {
                VS5.setPower(0);
                HL6.setPower(0);
            }






            lastDpadUp1 = gamepad1.dpad_up;
            lastDpadUp2 = gamepad2.dpad_up;
            lastDpadRight1 = gamepad1.dpad_right;




            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("HL6 Power", "%.2f", hl6Power);
            telemetry.addData("VS5 Power", "%.2f", vs5Power);
            telemetry.update();
        }
    }
}}
    }
}
// 4 pinpoint
// 2 baterias
// 10 rev NEO 2.0
// 2 sensores de cor
// 10 motesres rev
// 6 four bar odometry pod gobilda
//


