/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoE extends LinearOpMode {

    double COUNTS_PER_MOTOR_REV = 21; // Definição de counts para conversão da movimentação em CM)
    double DRIVE_GEAR_REDUCTION = 20.8; // Redução da engrenagem
    double DIAMETRO_RODA = 9.6; // Diâmetro da roda (Mecanum GoBilda)
    double circunferencia = DIAMETRO_RODA * Math.PI; // Conversão da circunfência com Diâmetro da roda com valor de PI)

    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel red;
    private DigitalChannel green;
    private DigitalChannel red1;
    private DigitalChannel green1;
    private DigitalChannel red2;
    private DigitalChannel green2;
    private DigitalChannel red3;
    private DigitalChannel green3;

    DcMotor motor_ef;
    DcMotor motor_et;
    DcMotor motor_df;
    DcMotor motor_dt;

    DcMotor corredicaD;

    DcMotor ColetorD;
    DcMotor ColetorE;

    Servo linear;
    Servo linear1;

    ColorSensor sensorled;

    Servo dropper;
    double CountsMotor = 28 ;
    double ReducaoEngranagem = 20 ;
    double rodaDiametro = 3.78;
    double countCM = (CountsMotor * ReducaoEngranagem) / (rodaDiametro * Math.PI);

    ElapsedTime relogio = new ElapsedTime();


    BNO055IMU imu; // Declaração do Giroscópio

    Orientation angles; // Orientação da angulação do giroscópio

//=====================================DECLARANDO OBJETOS DA CÂMERA===========================================================

    OpenCvCamera camera; //declarando a câmera
    Pipeline aprilTagDetectionPipeline; //definindo as marcações que identificam o objeto

    static final double FEET_PER_METER = 3.28084;

    // configuração da câmera
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // Unidades são metros
    double tagsize = 0.166;

    // declaração das imgaens 1, 2 e 3
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");

        red1 = hardwareMap.get(DigitalChannel.class, "red1");
        green1 = hardwareMap.get(DigitalChannel.class, "green1");

        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        green2 = hardwareMap.get(DigitalChannel.class, "green2");

        red3 = hardwareMap.get(DigitalChannel.class, "red3");
        green3 = hardwareMap.get(DigitalChannel.class, "green3");

        ColetorE = hardwareMap.get(DcMotor.class, "coletorE");
        ColetorE.setDirection(DcMotorSimple.Direction.REVERSE);
        ColetorD = hardwareMap.get(DcMotor.class, "coletorD");


        motor_df = hardwareMap.get(DcMotor.class,"frente_d");
        motor_ef = hardwareMap.get(DcMotor.class,"frente_e");
        motor_dt = hardwareMap.get(DcMotor.class,"tras_d");
        motor_et = hardwareMap.get(DcMotor.class,"tras_e");

        motor_df.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // definição de modo de parada dos motores
        motor_ef.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_et.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_ef.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_et.setDirection(DcMotorSimple.Direction.REVERSE);

        corredicaD = hardwareMap.get(DcMotor.class, "slideD");
        corredicaD.setDirection(DcMotorSimple.Direction.FORWARD);

        corredicaD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        corredicaD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        corredicaD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corredicaD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dropper = hardwareMap.get(Servo.class, "servo");

        linear = hardwareMap.get(Servo.class, "linear");
        linear.setDirection(Servo.Direction.FORWARD);

        linear1 = hardwareMap.get(Servo.class, "linear1");

//----------------------------------ALGORITIMO DE IDENTIFICAÇÃO DE IMAGEM---------------------------------------------------------

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new Pipeline(tagsize, fx, fy, cx, cy); //identificação do pipeline

        camera.setPipeline(aprilTagDetectionPipeline); //setup da câmera para identificar somente o pipeline
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() // criação do método de identificação
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT); //rastreamento do objeto
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        imu.initialize(parameters);
        while (!isStarted() && !isStopRequested())
        {
            red.setMode(DigitalChannel.Mode.OUTPUT);
            green.setMode(DigitalChannel.Mode.OUTPUT);
            red1.setMode(DigitalChannel.Mode.OUTPUT);
            green1.setMode(DigitalChannel.Mode.OUTPUT);

            red2.setMode(DigitalChannel.Mode.OUTPUT);
            green2.setMode(DigitalChannel.Mode.OUTPUT);
            red3.setMode(DigitalChannel.Mode.OUTPUT);
            green3.setMode(DigitalChannel.Mode.OUTPUT);

            red.setState(true);
            green.setState(false);
            red1.setState(true);
            green1.setState(false);

            red2.setState(true);
            green2.setState(false);
            red3.setState(true);
            green3.setState(false);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajetória 1
            dropper.setPosition(0.7);
            sleep(100);
            linear.setPosition(0.10);
            linear1.setPosition(0.10);
            encoder(0.7, -110, -110, 5, 0, "T");
            ColetorE.setPower(1);
            ColetorD.setPower(1);
            sleep(200);
            SET_SLIDE_POSITION("7", 1, false);
            ColetorE.setPower(0);
            ColetorD.setPower(0);
            IMU(138);
            sleep(100);
            encoder(0.3, 27, 27, 5, 138, "F");
            sleep(1000);
            dropper.setPosition(0.45);
            sleep(700);
            encoder(0.7, -17, -17, 5, 138, "T");
            SET_SLIDE_POSITION("6", 1, false);
            IMU(-95);
            sleep(100);
            encoder(0.8, 58, 58, 6, -95, "F");
            encoder(0.4, 11, 11, 1, -95, "F");
            SET_SLIDE_POSITION("5", 0.7, true);
            sleep(300);
            dropper.setPosition(0.7);
            sleep(500);
            SET_SLIDE_POSITION("6", 0.7, true);
            encoder(0.7, -55, -55, 5, -87, "T");
            sleep(100);
            IMU(136);
            SET_SLIDE_POSITION("8", 1, false);
            sleep(100);

            encoder(0.3, 24, 24, 5, 134, "F");
            sleep(1000);
            dropper.setPosition(0.45);
            sleep(500);
            encoder(0.8, -9, -9, 5, 144, "T");
            IMU(170);
            dropper.setPosition(0.7);
            sleep(100);
            MOVE_WITH_ENCODERS(motor_ef, motor_et, motor_df, motor_dt, imu, 96, 1, -135, 7 ,"E");
            SET_SLIDE_POSITION("1", 1, false);


        } else if (tagOfInterest.id == MIDDLE){
            //trajetória 2
            dropper.setPosition(0.7);
            sleep(100);
            linear.setPosition(0.10);
            linear1.setPosition(0.10);
            encoder(0.7, -110, -110, 5, 0, "T");
            ColetorE.setPower(1);
            ColetorD.setPower(1);
            sleep(200);
            SET_SLIDE_POSITION("7", 1, false);
            ColetorE.setPower(0);
            ColetorD.setPower(0);
            IMU(138);
            sleep(100);
            encoder(0.3, 27, 27, 5, 138, "F");
            sleep(1000);
            dropper.setPosition(0.45);
            sleep(700);
            encoder(0.7, -17, -17, 5, 138, "T");
            SET_SLIDE_POSITION("6", 1, false);
            IMU(-95);
            sleep(100);
            encoder(0.8, 58, 58, 6, -95, "F");
            encoder(0.4, 11, 11, 1, -95, "F");
            SET_SLIDE_POSITION("5", 0.7, true);
            sleep(300);
            dropper.setPosition(0.7);
            sleep(500);
            SET_SLIDE_POSITION("6", 0.7, true);
            encoder(0.7, -55, -55, 5, -87, "T");
            sleep(100);
            IMU(136);
            SET_SLIDE_POSITION("8", 1, false);
            sleep(100);

            encoder(0.3, 24, 24, 5, 134, "F");
            sleep(1000);
            dropper.setPosition(0.45);
            sleep(500);
            encoder(0.8, -9, -9, 5, 144, "T");
            IMU(170);
            dropper.setPosition(0.7);
            sleep(100);
           // MOVE_WITH_ENCODERS(motor_ef, motor_et, motor_df, motor_dt, imu, 96, 1, -135, 7 ,"E");
            SET_SLIDE_POSITION("1", 1, false);

        } else {
            //trajetória 3
           // encoder(0.5, 20, 20, 0.5, 0, "T");
            dropper.setPosition(0.7);
            sleep(100);
            linear.setPosition(0.10);
            linear1.setPosition(0.10);
            encoder(0.7, -110, -110, 5, 0, "T");
            ColetorE.setPower(1);
            ColetorD.setPower(1);
            sleep(200);
            SET_SLIDE_POSITION("7", 1, false);
            ColetorE.setPower(0);
            ColetorD.setPower(0);
            IMU(138);
            sleep(100);
            encoder(0.3, 27, 27, 5, 138, "F");
            sleep(1000);
            dropper.setPosition(0.45);
            sleep(700);
            encoder(0.7, -17, -17, 5, 138, "T");
            SET_SLIDE_POSITION("6", 1, false);
            IMU(-95);
            sleep(100);
            encoder(0.8, 58, 58, 6, -95, "F");
            encoder(0.4, 11, 11, 1, -95, "F");
            SET_SLIDE_POSITION("5", 0.7, true);
            sleep(300);
            dropper.setPosition(0.7);
            sleep(500);
            SET_SLIDE_POSITION("6", 0.7, true);
            encoder(0.7, -55, -55, 5, -87, "T");
            sleep(100);
            IMU(136);
            SET_SLIDE_POSITION("8", 1, false);
            sleep(100);

            encoder(0.3, 24, 24, 5, 134, "F");
            sleep(1000);
            dropper.setPosition(0.45);
            sleep(500);
            encoder(0.8, -9, -9, 5, 144, "T");
            IMU(170);
            dropper.setPosition(0.7);
            sleep(100);
            MOVE_WITH_ENCODERS(motor_ef, motor_et, motor_df, motor_dt, imu, 88, 1, -135, 7 ,"D");
            SET_SLIDE_POSITION("1", 1, false);

        }
        while (opModeIsActive()) {}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("angle", angles);
    }

    //-----------------CRIAÇÃO DO MÉTODO ENCODER E CURVA COM GIRO--------------------------------------------
    public void SET_SLIDE_POSITION(String level, double power, boolean whileMode) {
        corredicaD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        switch (level) {
            case "1":
                corredicaD.setTargetPosition(0);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
            case "2":
                corredicaD.setTargetPosition(200);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
            case "3":
                corredicaD.setTargetPosition(300);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
            case "4":
                corredicaD.setTargetPosition(410);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
            case "5":
                corredicaD.setTargetPosition(350);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
            case "6":
                corredicaD.setTargetPosition(1450);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
            case "7":
                corredicaD.setTargetPosition(2880);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (whileMode) {
                    while (corredicaD.isBusy()){
                        corredicaD.setPower(power);
                    }
                } else {
                    corredicaD.setPower(power);
                }

                break;
            case "8":
                corredicaD.setTargetPosition(2900);
                corredicaD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (corredicaD.isBusy()){
                    corredicaD.setPower(power);
                }
                break;
        }
    }

    public void encoder(double veloc, double leftCM, double rightCM, double timeoutS, int angle, String direcao) {


        int rightPos_F;
        int leftPos_F;
        int rightPos_B;
        int leftPos_B;
        double rightCM_N = -rightCM;
        double leftCM_N = -leftCM;
        motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_ef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_df.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_et.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_dt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive()) {

            if (direcao.equals("T")) {
                rightPos_F = (int) (rightCM / 2.54 * countCM);
                leftPos_F = (int) (leftCM / 2.54 * countCM);
                motor_ef.setTargetPosition(leftPos_F);
                motor_df.setTargetPosition(rightPos_F);
                motor_et.setTargetPosition(leftPos_F);
                motor_dt.setTargetPosition(rightPos_F);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                relogio.reset();

                while (opModeIsActive() && (relogio.seconds() < timeoutS) &&
                        (motor_ef.isBusy() && motor_df.isBusy() &&
                                motor_et.isBusy() && motor_dt.isBusy())) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    int z = Math.round(angles.firstAngle);

                    if(z > angle) {
                        motor_ef.setPower(Math.abs(veloc));
                        motor_df.setPower(Math.abs(veloc - 0.2));
                        motor_et.setPower(Math.abs(veloc));
                        motor_dt.setPower(Math.abs(veloc - 0.2));
                    }
                    if (z < angle) {
                        motor_ef.setPower(Math.abs(veloc - 0.2));
                        motor_df.setPower(Math.abs(veloc ));
                        motor_et.setPower(Math.abs(veloc - 0.2));
                        motor_dt.setPower(Math.abs(veloc));
                    } else {
                        motor_ef.setPower(Math.abs(veloc));
                        motor_df.setPower(Math.abs(veloc));
                        motor_et.setPower(Math.abs(veloc));
                        motor_dt.setPower(Math.abs(veloc));
                    }
                }

                motor_ef.setPower(0);
                motor_df.setPower(0);
                motor_et.setPower(0);
                motor_dt.setPower(0);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor_ef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_df.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_et.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            } else if (direcao.equals("F")) {
                rightPos_F = (int) (rightCM / 2.54 * countCM);
                leftPos_F = (int) (leftCM / 2.54 * countCM);
                motor_ef.setTargetPosition(leftPos_F);
                motor_df.setTargetPosition(rightPos_F);
                motor_et.setTargetPosition(leftPos_F);
                motor_dt.setTargetPosition(rightPos_F);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                relogio.reset();

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                int z = Math.round(angles.firstAngle);

                while (opModeIsActive() && (relogio.seconds() < timeoutS) &&
                        (motor_ef.isBusy() && motor_df.isBusy() &&
                                motor_et.isBusy() && motor_dt.isBusy())) {

                    if (z < angle) {
                        motor_ef.setPower(Math.abs(veloc));
                        motor_df.setPower(Math.abs(veloc - 0.2));
                        motor_et.setPower(Math.abs(veloc));
                        motor_dt.setPower(Math.abs(veloc - 0.2));
                    } else if (z > angle) {
                        motor_ef.setPower(Math.abs(veloc - 0.2));
                        motor_df.setPower(Math.abs(veloc ));
                        motor_et.setPower(Math.abs(veloc - 0.2));
                        motor_dt.setPower(Math.abs(veloc));
                    } else {
                        motor_ef.setPower(Math.abs(veloc));
                        motor_df.setPower(Math.abs(veloc));
                        motor_et.setPower(Math.abs(veloc));
                        motor_dt.setPower(Math.abs(veloc));
                    }
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    z = Math.round(angles.firstAngle);
                }

                motor_ef.setPower(0);
                motor_df.setPower(0);
                motor_et.setPower(0);
                motor_dt.setPower(0);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor_ef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_df.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_et.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else if (direcao.equals("E")){
                rightPos_F = (int) (rightCM / 2.54 * countCM);
                leftPos_F = (int) (leftCM / 2.54 * countCM);
                rightPos_B = (int) (rightCM_N / 2.54 * countCM);
                leftPos_B = (int) (leftCM_N / 2.54 * countCM);

                motor_ef.setTargetPosition(leftPos_B);
                motor_df.setTargetPosition(rightPos_F);
                motor_et.setTargetPosition(leftPos_F);
                motor_dt.setTargetPosition(rightPos_B);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                relogio.reset();

                while (opModeIsActive() && (relogio.seconds() < timeoutS) &&
                        (motor_ef.isBusy() && motor_df.isBusy() &&
                                motor_et.isBusy() && motor_dt.isBusy())) {

                    motor_ef.setPower(Math.abs(veloc));
                    motor_df.setPower(Math.abs(veloc));
                    motor_et.setPower(Math.abs(veloc));
                    motor_dt.setPower(Math.abs(veloc));
                }

                motor_ef.setPower(0);
                motor_df.setPower(0);
                motor_et.setPower(0);
                motor_dt.setPower(0);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor_ef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_df.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_et.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }else{
                rightPos_F = (int) (rightCM / 2.54 * countCM);
                leftPos_F = (int) (leftCM / 2.54 * countCM);
                rightPos_B = (int) (rightCM_N / 2.54 * countCM);
                leftPos_B = (int) (leftCM_N / 2.54 * countCM);

                motor_ef.setTargetPosition(leftPos_F);
                motor_df.setTargetPosition(rightPos_B);
                motor_et.setTargetPosition(leftPos_B);
                motor_dt.setTargetPosition(rightPos_F);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                relogio.reset();

                while (opModeIsActive() && (relogio.seconds() < timeoutS) &&
                        (motor_ef.isBusy() && motor_df.isBusy() &&
                                motor_et.isBusy() && motor_dt.isBusy())) {

                    motor_ef.setPower(Math.abs(veloc));
                    motor_df.setPower(Math.abs(veloc));
                    motor_et.setPower(Math.abs(veloc));
                    motor_dt.setPower(Math.abs(veloc));
                }

                motor_ef.setPower(0);
                motor_df.setPower(0);
                motor_et.setPower(0);
                motor_dt.setPower(0);

                motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor_ef.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_df.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_et.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
    void IMU (double angle){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int z = Math.round(angles.firstAngle);
        if(z < angle) {
            while (z <= angle){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                z = Math.round(angles.firstAngle);
                motor_ef.setPower(0.8);
                motor_df.setPower(-0.8);
                motor_et.setPower(0.8);
                motor_dt.setPower(-0.8);
            }
            motor_ef.setPower(0);
            motor_df.setPower(0);
            motor_et.setPower(0);
            motor_dt.setPower(0);
            sleep(200);
            while (z >= angle){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                z = Math.round(angles.firstAngle);
                motor_ef.setPower(-0.2);
                motor_df.setPower(0.2);
                motor_et.setPower(-0.2);
                motor_dt.setPower(0.2);
            }
        }
        if (z > angle) {
            while (z >= angle){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                z = Math.round(angles.firstAngle);
                motor_ef.setPower(-0.8);
                motor_df.setPower(0.8);
                motor_et.setPower(-0.8);
                motor_dt.setPower(0.8);
            }
            motor_ef.setPower(0);
            motor_df.setPower(0);
            motor_et.setPower(0);
            motor_dt.setPower(0);
            sleep(200);
            while (z <= angle){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                z = Math.round(angles.firstAngle);
                motor_ef.setPower(0.2);
                motor_df.setPower(-0.2);
                motor_et.setPower(0.2);
                motor_dt.setPower(-0.2);
            }
        }
        motor_ef.setPower(0);
        motor_df.setPower(0);
        motor_et.setPower(0);
        motor_dt.setPower(0);
    }


    public void MOVE_WITH_ENCODERS(DcMotor motor_ef, DcMotor motor_et, DcMotor motor_df, DcMotor motor_dt, BNO055IMU imu,
                                   double target, double power, double angle, double timeout, String direction) {

        int targetPosition = (int)Math.round(target * (DRIVE_GEAR_REDUCTION * COUNTS_PER_MOTOR_REV) / circunferencia);

        SET_MODE_STOP_AND_RESET_ENCODER(motor_ef, motor_et, motor_df, motor_dt);
        SET_MODE_RUN_USING_ENCODER(motor_ef, motor_et, motor_df, motor_dt);

        if (opModeIsActive()) {
            switch (direction) {
                case "F":
                    motor_ef.setTargetPosition(targetPosition);
                    motor_et.setTargetPosition(targetPosition);
                    motor_df.setTargetPosition(targetPosition);
                    motor_dt.setTargetPosition(targetPosition);

                    SET_MODE_RUN_TO_POSITION(motor_ef, motor_et, motor_df, motor_dt);

                    RUN_MOTORS_WITH_IMU_PID_CONTROLLER(motor_ef, motor_et, motor_df, motor_dt, imu, power, angle, timeout, direction);

                    SET_MODE_STOP_AND_RESET_ENCODER(motor_ef, motor_et, motor_df, motor_dt);
                    break;

                case "T":
                    motor_ef.setTargetPosition(-targetPosition);
                    motor_et.setTargetPosition(-targetPosition);
                    motor_df.setTargetPosition(-targetPosition);
                    motor_dt.setTargetPosition(-targetPosition);

                    SET_MODE_RUN_TO_POSITION(motor_ef, motor_et, motor_df, motor_dt);

                    RUN_MOTORS_WITH_IMU_PID_CONTROLLER(motor_ef, motor_et, motor_df, motor_dt, imu, power, angle, timeout, direction);

                    SET_MODE_STOP_AND_RESET_ENCODER(motor_ef, motor_et, motor_df, motor_dt);
                    break;

                case "E":
                    motor_ef.setTargetPosition(-targetPosition);
                    motor_et.setTargetPosition(targetPosition);
                    motor_df.setTargetPosition(targetPosition);
                    motor_dt.setTargetPosition(-targetPosition);

                    SET_MODE_RUN_TO_POSITION(motor_ef, motor_et, motor_df, motor_dt);

                    runtime.reset();

                    while (opModeIsActive() && (runtime.seconds() < timeout) &&
                            (motor_ef.isBusy() && motor_et.isBusy() && motor_df.isBusy() && motor_dt.isBusy())) {

                        motor_ef.setPower(Math.abs(power));
                        motor_et.setPower(Math.abs(power));
                        motor_df.setPower(Math.abs(power));
                        motor_dt.setPower(Math.abs(power));

                    }

                    SET_MODE_STOP_AND_RESET_ENCODER(motor_ef, motor_et, motor_df, motor_dt);
                    break;

                case "D":
                    motor_ef.setTargetPosition(targetPosition);
                    motor_et.setTargetPosition(-targetPosition);
                    motor_df.setTargetPosition(-targetPosition);
                    motor_dt.setTargetPosition(targetPosition);

                    SET_MODE_RUN_TO_POSITION(motor_ef, motor_et, motor_df, motor_dt);

                    runtime.reset();

                    while (opModeIsActive() && (runtime.seconds() < timeout) &&
                            (motor_ef.isBusy() && motor_et.isBusy() && motor_df.isBusy() && motor_dt.isBusy())) {
                        motor_ef.setPower(Math.abs(power));
                        motor_et.setPower(Math.abs(power));
                        motor_df.setPower(Math.abs(power));
                        motor_dt.setPower(Math.abs(power));
                    }

                    SET_MODE_STOP_AND_RESET_ENCODER(motor_ef, motor_et, motor_df, motor_dt);
                    break;
            }
        }
    }


    public void RUN_MOTORS_WITH_IMU_PID_CONTROLLER(DcMotor motor_ef, DcMotor motor_et, DcMotor motor_df, DcMotor motor_dt, BNO055IMU imu, double power, double angle, double timeout, String direction) {

        double Kp = 0.03;
        double Ki = 0.004;
        double Kd = 0;

        double integralSum = 0;
        double integralSumLimit = 0.35;

        double outLimit = 0.4;

        double lastError = 0;

        double lastReference = angle;

        double a = 0.8; // 0 < a < 1
        double currentFilterEstimate;
        double previousFilterEstimate = 0;

        ElapsedTime timer = new ElapsedTime();

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timeout) &&
                (motor_ef.isBusy() && motor_et.isBusy() && motor_df.isBusy() && motor_dt.isBusy())) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle - Math.round(angles.firstAngle);
            double errorChange = (error - lastError);

            currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            double derivative = currentFilterEstimate  / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            if (integralSum > integralSumLimit) {
                integralSum = integralSumLimit;
            }
            if (integralSum < -integralSumLimit) {
                integralSum = -integralSumLimit;
            }

            if (angle != lastReference) {
                integralSum = 0;
            }

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (out > outLimit) {
                out = outLimit;
            }

            if (out < -outLimit) {
                out = -outLimit;
            }

            telemetry.addData("Output", out);
            telemetry.addData("CurrentAngle", Math.round(angles.firstAngle));
            telemetry.addData("angle", angle);
            telemetry.update();

            switch (direction) {
                case "F":
                    motor_ef.setPower(power + out);
                    motor_et.setPower(power + out);
                    motor_df.setPower(power - out);
                    motor_dt.setPower(power - out);
                    break;

                case "T":
                    motor_ef.setPower(power - out);
                    motor_et.setPower(power - out);
                    motor_df.setPower(power + out);
                    motor_dt.setPower(power + out);
                    break;
            }

            lastError = error;
            lastReference = angle;

            timer.reset();
        }
    }


    public void SET_MODE_STOP_AND_RESET_ENCODER(DcMotor motor_ef, DcMotor motor_et, DcMotor motor_df, DcMotor motor_dt) {
        motor_ef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_et.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_df.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_dt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void SET_MODE_RUN_USING_ENCODER(DcMotor motor_ef, DcMotor motor_et, DcMotor motor_df, DcMotor motor_dt) {
        motor_ef.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_et.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_df.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_dt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SET_MODE_RUN_TO_POSITION(DcMotor motor_ef, DcMotor motor_et, DcMotor motor_df, DcMotor motor_dt) {
        motor_ef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_et.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_df.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_dt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}