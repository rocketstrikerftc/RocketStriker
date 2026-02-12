package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "rathian")
public class test extends OpMode {

    private Limelight3A limelight;
    TestBench bench = new TestBench();
    private double distance;

    // ---- ADIÇÃO: variáveis para calibração automática ----
    private final List<Double> knownDistances = new ArrayList<>();
    private final List<Double> measuredAreas = new ArrayList<>();
    private boolean calibrating = false;
    private double calibratedK = 179.0; // valor inicial padrão
    // -------------------------------------------------------

    @Override
    public void init() {
        bench.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight pronta. Pressione [A] para modo de calibração.");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // ---- ADIÇÃO: controle de calibração via botão ----
        if (gamepad1.a) {
            calibrating = true;
            telemetry.addLine("MODO CALIBRAÇÃO ATIVADO! Coloque o robô em distâncias conhecidas e pressione [X] para registrar.");
        }
        if (gamepad1.b) {
            calibrating = false;
            telemetry.addLine("MODO CALIBRAÇÃO DESATIVADO.");
        }
        // ---------------------------------------------------

        YawPitchRollAngles orientation = bench.getOrientation();
        if (orientation != null)
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            double ta = llResult.getTa();

            // ---- ADIÇÃO: registro de pontos de calibração ----
            if (calibrating && gamepad1.x) {
                double knownDistance = askDistanceFromUser(); // define manualmente abaixo
                knownDistances.add(knownDistance);
                measuredAreas.add(ta);
                telemetry.addData("Registrado", "Dist=%.1f cm | ta=%.4f", knownDistance, ta);
            }

            // Atualiza K automaticamente se houver dados
            if (knownDistances.size() >= 2) {
                calibratedK = computeCalibrationConstant();
            }
            // ---------------------------------------------------

            distance = getDistanceFromTage(ta);
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", ta);
            telemetry.addData("Calibrated K", calibratedK);
            telemetry.addData("Botpose", botpose.toString());
        }

        telemetry.update();
    }

    // ---- ADIÇÃO: cálculo automático da constante K ----
    private double computeCalibrationConstant() {
        double sum = 0;
        for (int i = 0; i < knownDistances.size(); i++) {
            double d = knownDistances.get(i);
            double ta = measuredAreas.get(i);
            if (ta > 0) sum += d * Math.sqrt(ta);
        }
        return sum / knownDistances.size();
    }

    // ---- ADIÇÃO: valor manual de calibração (coloque sua medição real) ----
    private double askDistanceFromUser() {
        // ⚠️ Aqui você pode colocar manualmente o valor da distância medida (em cm)
        // quando apertar [X] durante o modo calibração.
        // Exemplo: retorne 50 para 50 cm, depois mude pra 100, etc.
        return 100; // <<< ALTERE AQUI quando calibrar
    }
    // -----------------------------------------------------------------------

    public double getDistanceFromTage(double ta) {
        if (ta <= 0) return 0;
        double distance = calibratedK / Math.sqrt(ta);
        return distance;
    }
}