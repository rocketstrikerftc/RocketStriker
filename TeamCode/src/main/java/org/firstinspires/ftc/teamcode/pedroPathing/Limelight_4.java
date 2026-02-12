/*
Copyright (c) 2024 Limelight Vision
All rights reserved.
*/

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@Disabled
@TeleOp(name = "Loucura")
public class Limelight_4 extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // Pipeline 0 = AprilTags
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            int currentPipeline = status.getPipelineIndex();
            if (currentPipeline == 0) telemetry.addLine("🟢 Pipeline 0 ativa - Detectando AprilTags");

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

                        // Telemetry estilo "AprilTagDetection"
                        telemetry.addData("Tag ID", tagId);
                        telemetry.addData("Family", family);
                        telemetry.addData("Position", String.format("X: %.2f°, Y: %.2f°", x, y));

                        // Ações diferentes por ID
                        switch (tagId) {
                            case 21:
                                telemetry.addLine("→ AÇÃO para tag 21");
                                break;
                            case 22:
                                telemetry.addLine("→ AÇÃO para tag 22");
                                break;
                            case 23:
                                telemetry.addLine("→ AÇÃO para tag 23");
                                break;
                            default:
                                telemetry.addLine("→ Outra tag detectada: " + tagId);
                                break;
                        }
                    }
                }

            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
