package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Disabled
@TeleOp(name = "slim")
public class sim extends LinearOpMode {

    public Limelight3A limelight;
    public DcMotor FL0;



    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        FL0 = hardwareMap.get(DcMotor.class,"FL0");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

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



            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Informações gerais
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                result.getPipelineIndex();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());


                // Barcode
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Classifier
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Detector
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }


                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                    if (!fiducialResults.isEmpty()) {
                        int actionID = -1;

                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            int id = fr.getFiducialId();
                            if (id == 21) {
                                actionID = 21;
                                break;
                            } else if (id == 22 && actionID != 21) {
                                actionID = 22;
                            } else if (id == 23 && actionID != 21 && actionID != 22) {
                                actionID = 23;
                            }
                        }

                        switch (actionID) {
                            case 21:
                                FL0.setPower(0.2);
                                sleep(200);
                                FL0.setPower(0);
                                sleep(5000);
                                stop();


                                telemetry.addLine(" ação 1");
                                break;

                            case 22:
                                FL0.setPower(-0.3);
                                sleep(200);
                                FL0.setPower(0);
                                sleep(1000);
                                FL0.setPower(0.2);
                                sleep(200);
                                FL0.setPower(0);
                                sleep(3000);
                                telemetry.addLine(" ação 2");
                                break;

                            case 23:
                                FL0.setPower(-0.2);
                                sleep(200);
                                FL0.setPower(0);
                                sleep(1000);
                                FL0.setPower(0.2);
                                sleep(200);
                                FL0.setPower(0);
                                sleep(3000);

                                telemetry.addLine("ação 3");
                                break;
                            default:
                                FL0.setPower(0);
                                telemetry.addLine("Nenhuma tag encontrda");
                                break;
                        }

                    } else {
                        FL0.setPower(0);
                        telemetry.addLine("Nenhum fiducial encontrado");
                    }
                }



                telemetry.update();



        }


    }

}}
