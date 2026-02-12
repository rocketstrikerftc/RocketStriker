package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp(name = "Limelight", group = "Test")
public class pidservo extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo cservo;

    // --- Constantes PID ---
    private double kP = 0.1;   // ganho proporcional → quanto reage ao erro
    private double kD = 0.01;  // ganho derivativo → suaviza e evita overshoot

    // --- Ajustes gerais ---
    private final double maxPower = 1.0;   // limite máximo de velocidade do servo
    private final double deadband = 1.0;   // margem de erro onde o servo para
    private final double filtroAlpha = 0.8; // suavização do tx (0 = lento, 1 = sem filtro)

    // --- Variáveis internas ---
    private double lastTx = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        cservo = hardwareMap.get(CRServo.class, "cservo");

        telemetry.addLine("Limelight Rápida e Precisa pronta!");
        telemetry.update();

        waitForStart();
        limelight.start();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();

                // --- Filtro de suavização ---
                double filteredTx = (1 - filtroAlpha) * lastTx + filtroAlpha * tx;
                lastTx = filteredTx;

                // --- Controle PID (apenas P + D) ---
                double currentTime = getRuntime();
                double deltaTime = currentTime - lastTime;
                lastTime = currentTime;

                double error = filteredTx;
                double derivativo = (error - lastError) / Math.max(deltaTime, 0.01);
                lastError = error;

                double power = (error * kP) + (derivativo * kD);

                // --- Zona morta ---
                if (Math.abs(filteredTx) < deadband) {
                    power = 0;
                }

                // --- Limita potência ---
                power = Math.max(-maxPower, Math.min(maxPower, power));

                // --- Inverte sinal se necessário ---
                cservo.setPower(-power);

                // --- Telemetria ---
                telemetry.addData("Status", Math.abs(filteredTx) < deadband ? "Centralizado" : "Rastreando");
                telemetry.addData("Erro (tx)", "%.2f", tx);
                telemetry.addData("Erro filtrado", "%.2f", filteredTx);
                telemetry.addData("Power", "%.3f", power);
                telemetry.addData("kP", kP);
                telemetry.addData("kD", kD);
                telemetry.addData("DeltaTime", "%.3f", deltaTime);
                telemetry.addData("Derivativo", "%.3f", derivativo);
            } else {
                cservo.setPower(0);
                telemetry.addData("Tag Detectada", false);
            }

            telemetry.update();
        }
    }
}