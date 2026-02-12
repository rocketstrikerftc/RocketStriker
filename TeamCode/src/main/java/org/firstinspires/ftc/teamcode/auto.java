package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Azul frente")
public class auto extends OpMode {

    private static final double SHOOT_ON_TIME = 0.2;   // 200 ms
    private static final double SHOOT_OFF_TIME = 0.4;  // 400 ms
    private static final int SHOOT_CYCLES = 3;

    public DcMotor SH4;
    public DcMotor VS5;
    public DcMotor HL6;
    public DcMotor LC7;
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
    DRIVE_STARTPOS_SHOOT_POS,
    SHOOT_PRELOAD,
    DRIVE_SHOOTPOS_ARTEFATOPOS,
        DRIVE_ARTEFATOPOS_PEGA1,
        DRIVE_PEGA1_SHOOT2,
    DRIVE_SHOOT2POS_ARTEFATO2POS,
    DRIVE_ARTEFATO2POS_GATEPOS,
        DRIVE_ARTEFATO2POS_GATE3POS,
        DRIVE_GATE3POS_SHOOT3POS,
        DRIVE_SHOOT3POS_SHOOT4POS,
        DRIVE_SHOOT4POS_ARTEFATO3POS,
        DRIVE_ARTEFATO3POS_SHOOT5POS,
        DRIVE_SHOOT5POS_ARTEFATO4POS,
        DRIVE_ARTEFATO4POS_PEGA2POS,
        DRIVE_PEGA2POS_SHOOT6POS,

        DRIVE_INTAKE,
        DRIVE_INTAKE1,
        DRIVE_INTAKE2,
        DRIVE_INTAKE3,
        SHOOT2,
        SHOOT3,
        SHOOT4,
        SHOOT5,
        SHOOT6,




        DRIVE_PARA;

    }

    PathState pathState;

    private final Pose startPose = new Pose(27.221, 130.297,Math.toRadians(143));
    private final Pose shootpose1 = new Pose(47.040, 97.567,Math.toRadians(141));
    private final Pose artefato1 = new Pose(42.7,84.9,Math.toRadians(180));
    private final Pose pega1 = new Pose(19.800,84.400,Math.toRadians(180));
    private final Pose shoot2 = new Pose(44.644,92.410,Math.toRadians(145));
    private final Pose artefato2 = new Pose(42.594,60.676,Math.toRadians(180));
    private final Pose gate = new Pose(21.0,60.800,Math.toRadians(90));
    private final Pose gate2 = new Pose(25.915,71.424,Math.toRadians(90));
    private final Pose gate3 = new Pose(11.245,70.126,Math.toRadians(90));
    private final Pose shoot3 = new Pose(53.700,82.200,Math.toRadians(140));
    private final Pose shoot4 = new Pose(41.214,36.317,Math.toRadians(180));
    private final Pose artefato3 = new Pose(21.634,35.415);
    private final Pose shoot5 = new Pose(49.341,11.707,Math.toRadians(115));
    private final Pose artefato4 = new Pose(9.221,29.452,Math.toRadians(270));
    private final Pose pega2 = new Pose(9.267,9.814);
    private final Pose beziercurve = new Pose(28.838,17.193,Math.toRadians(270));
    private final Pose beziercurve2 = new Pose(49.960,12.628,Math.toRadians(113));


    private PathChain driveStartPosShootPos, driveShootPosArtefato1Pos, driveArtefato1PosPega1Pos, drivePega1PosShoot2Pos, driveShoot2PosArtefato2Pos, driveArtefato2PosGatePos,driveArtefato2PosGate3Pos,
            driveGate3PosShoot3Pos,driveShoot3PosShoot4,driveShoot4PosArtefato3Pos,driveArtefato3PosShoot5Pos,driveShoot5PosArteafto4Pos,driveArteafto4PosPega2Pos,drivePega2PosShoot6Pos;

    public void buildPaths() {

      driveStartPosShootPos = follower.pathBuilder()
              .addPath(new BezierLine(startPose, shootpose1))
              .setLinearHeadingInterpolation(startPose.getHeading(),shootpose1.getHeading())
              .build();
        driveShootPosArtefato1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootpose1, artefato1))
                .setLinearHeadingInterpolation(shootpose1.getHeading(),artefato1.getHeading())
                .build();
        driveArtefato1PosPega1Pos = follower.pathBuilder()
                .addPath(new BezierLine(artefato1, pega1))
                .setTangentHeadingInterpolation()
                .build();
        drivePega1PosShoot2Pos = follower.pathBuilder()
                .addPath(new BezierLine(pega1, shoot2))
                .setLinearHeadingInterpolation(pega1.getHeading(),shoot2.getHeading())
                .build();
        driveShoot2PosArtefato2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, artefato2))
                .setLinearHeadingInterpolation(shoot2.getHeading(),artefato2.getHeading())
                .build();
        driveArtefato2PosGatePos = follower.pathBuilder()
                .addPath(new BezierCurve( artefato2,gate,gate2))
                .setLinearHeadingInterpolation(artefato2.getHeading(),gate2.getHeading())
                .build();
        driveArtefato2PosGate3Pos = follower.pathBuilder()
                .addPath(new BezierLine(gate2, gate3))
                .setLinearHeadingInterpolation(gate2.getHeading(),gate3.getHeading())
                .build();
        driveGate3PosShoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(gate3, shoot3))
                .setLinearHeadingInterpolation(gate3.getHeading(),shoot3.getHeading())
                .build();
        driveShoot3PosShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3, shoot4))
                .setLinearHeadingInterpolation(shoot3.getHeading(),shoot4.getHeading())
                .build();
        driveShoot4PosArtefato3Pos = follower.pathBuilder()
                .addPath(new BezierLine( shoot4,artefato3))
                .setTangentHeadingInterpolation()
                .build();
        driveArtefato3PosShoot5Pos = follower.pathBuilder()
                .addPath(new BezierLine(artefato3, shoot5))
                .setLinearHeadingInterpolation(artefato3.getHeading(),shoot5.getHeading())
                .build();
        driveShoot5PosArteafto4Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot5, artefato4))
                .setLinearHeadingInterpolation(shoot5.getHeading(),artefato4.getHeading())
                .build();

        driveArteafto4PosPega2Pos = follower.pathBuilder()
                .addPath(new BezierLine(artefato4, pega2))
                .setTangentHeadingInterpolation()
                .build();
        drivePega2PosShoot6Pos = follower.pathBuilder()
                .addPath(new BezierCurve(pega2, beziercurve,beziercurve2))
                .setLinearHeadingInterpolation(shoot5.getHeading(),beziercurve2.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT2:
                follower.breakFollowing();
                if (!follower.isBusy()) {
                    double cycleTime = SHOOT_ON_TIME + SHOOT_OFF_TIME; // 0.6s
                    double totalTime = SHOOT_CYCLES * cycleTime;       // 1.8s

                    if (time < totalTime) {

                        double timeInCycle = time % cycleTime;

                        if (timeInCycle < SHOOT_ON_TIME) {
                            // 200 ms LIGADO
                            VS5.setPower(0.20);
                            HL6.setPower(0.42);
                        } else {
                            // 400 ms DESLIGADO
                            VS5.setPower(0);
                            HL6.setPower(0);
                        }

                    } else {
                        // Terminou os 3 ciclos
                        VS5.setPower(0);
                        HL6.setPower(0);

                        follower.followPath(drivePega1PosShoot2Pos, true);
                        setPathState(PathState.DRIVE_PEGA1_SHOOT2);
                    }
                }
                break;

            case SHOOT_PRELOAD:
                follower.breakFollowing();
                if (!follower.isBusy()) {
                    double cycleTime = SHOOT_ON_TIME + SHOOT_OFF_TIME; // 0.6s
                    double totalTime = SHOOT_CYCLES * cycleTime;       // 1.8s

                    if (time < totalTime) {

                        double timeInCycle = time % cycleTime;

                        if (timeInCycle < SHOOT_ON_TIME) {
                            // 200 ms LIGADO
                            VS5.setPower(0.20);
                            HL6.setPower(0.42);
                        } else {
                            // 400 ms DESLIGADO
                            VS5.setPower(0);
                            HL6.setPower(0);
                        }

                    } else {
                        // Terminou os 3 ciclos
                        VS5.setPower(0);
                        HL6.setPower(0);

                        follower.followPath(driveStartPosShootPos, true);
                        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
                    }
                }
                break;

            case SHOOT3:
                follower.breakFollowing();
                if (!follower.isBusy()) {
                    double cycleTime = SHOOT_ON_TIME + SHOOT_OFF_TIME; // 0.6s
                    double totalTime = SHOOT_CYCLES * cycleTime;       // 1.8s

                    if (time < totalTime) {

                        double timeInCycle = time % cycleTime;

                        if (timeInCycle < SHOOT_ON_TIME) {
                            // 200 ms LIGADO
                            VS5.setPower(0.20);
                            HL6.setPower(0.42);
                        } else {
                            // 400 ms DESLIGADO
                            VS5.setPower(0);
                            HL6.setPower(0);
                        }

                    } else {
                        // Terminou os 3 ciclos
                        VS5.setPower(0);
                        HL6.setPower(0);

                        follower.followPath(drivePega1PosShoot2Pos, true);
                        setPathState(PathState.DRIVE_SHOOT3POS_SHOOT4POS);
                    }
                }
                break;

            case SHOOT4:
                follower.breakFollowing();
                if (!follower.isBusy()) {
                    double cycleTime = SHOOT_ON_TIME + SHOOT_OFF_TIME; // 0.6s
                    double totalTime = SHOOT_CYCLES * cycleTime;       // 1.8s

                    if (time < totalTime) {

                        double timeInCycle = time % cycleTime;

                        if (timeInCycle < SHOOT_ON_TIME) {
                            // 200 ms LIGADO
                            VS5.setPower(0.20);
                            HL6.setPower(0.42);
                        } else {
                            // 400 ms DESLIGADO
                            VS5.setPower(0);
                            HL6.setPower(0);
                        }

                    } else {
                        // Terminou os 3 ciclos
                        VS5.setPower(0);
                        HL6.setPower(0);

                        follower.followPath(drivePega1PosShoot2Pos, true);
                        setPathState(PathState.DRIVE_PEGA1_SHOOT2);
                    }
                }
                break;
            case DRIVE_SHOOTPOS_ARTEFATOPOS:
                if (!follower.isBusy()) {
                    // Quando chegar no artefato, vai para a "pega"
                    follower.followPath(driveArtefato1PosPega1Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATOPOS_PEGA1);
                }
                break;

            case DRIVE_ARTEFATOPOS_PEGA1:
                if (!follower.isBusy()) {
                    // Após pegar, vai para o shoot 2
                    follower.followPath(drivePega1PosShoot2Pos, true);
                    setPathState(PathState.DRIVE_PEGA1_SHOOT2);
                }
                break;

            case DRIVE_PEGA1_SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot2PosArtefato2Pos, true);
                    setPathState(PathState.DRIVE_SHOOT2POS_ARTEFATO2POS);
                }
                break;

            case DRIVE_SHOOT2POS_ARTEFATO2POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveArtefato2PosGate3Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO2POS_GATEPOS);
                }
                break;

            case DRIVE_ARTEFATO2POS_GATE3POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveGate3PosShoot3Pos, true);
                    setPathState(PathState.DRIVE_GATE3POS_SHOOT3POS);
                }
                break;

            case DRIVE_GATE3POS_SHOOT3POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot3PosShoot4, true);
                    setPathState(PathState.DRIVE_SHOOT3POS_SHOOT4POS); // Corrigido nome do estado
                }
                break;

            case DRIVE_SHOOT3POS_SHOOT4POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot4PosArtefato3Pos, true);
                    setPathState(PathState.DRIVE_SHOOT4POS_ARTEFATO3POS); // Corrigido
                }
                break;

            // ... cases anteriores ...

            case DRIVE_SHOOT4POS_ARTEFATO3POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveArtefato3PosShoot5Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO3POS_SHOOT5POS); // Estava voltando pro anterior
                }
                break;

            case DRIVE_ARTEFATO3POS_SHOOT5POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot5PosArteafto4Pos, true);
                    setPathState(PathState.DRIVE_SHOOT5POS_ARTEFATO4POS); // Estava voltando pro anterior
                }
                break;

            case DRIVE_SHOOT5POS_ARTEFATO4POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveArteafto4PosPega2Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO4POS_PEGA2POS); // Corrigido
                }
                break;

            case DRIVE_ARTEFATO4POS_PEGA2POS:
                if (!follower.isBusy()) {
                    follower.followPath(drivePega2PosShoot6Pos, true);
                    setPathState(PathState.DRIVE_PEGA2POS_SHOOT6POS); // Corrigido
                }
                break;

            case DRIVE_PEGA2POS_SHOOT6POS:
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_PARA); // Crie um estado final para o robô parar
                }
                break;
        }
    }
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
    pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
    pathTimer = new Timer();
    opModeTimer = new Timer();
    follower = Constants.createFollower(hardwareMap);
        SH4 = hardwareMap.get(DcMotor.class, "SH4");
        VS5 = hardwareMap.get(DcMotor.class, "VS5");
        HL6 = hardwareMap.get(DcMotor.class, "HL6");
        LC7 = hardwareMap.get(DcMotor.class, "LC7");

        SH4.setDirection(DcMotor.Direction.FORWARD);
        VS5.setDirection(DcMotor.Direction.REVERSE);
        HL6.setDirection(DcMotor.Direction.FORWARD);
        LC7.setDirection(DcMotor.Direction.FORWARD);

    buildPaths();
    follower.setPose(startPose);
    }

    @Override
    public void start() {
    opModeTimer.resetTimer();
    setPathState(pathState);
    }

    @Override
    public void loop() {
    follower.update();
    statePathUpdate();

    telemetry.addData("path state",pathState.toString());
    telemetry.addData("x",follower.getPose().getX());
    telemetry.addData("y",follower.getPose().getY());
    telemetry.addData("heading",follower.getPose().getHeading());
    telemetry.addData("Path time",pathTimer.getElapsedTimeSeconds());



    }
}
