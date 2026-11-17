/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu''''''' of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear Opmode")
//@Disabled
public class teleOp extends LinearOpMode {

    private DigitalChannel red;
    private DigitalChannel green;
    private DigitalChannel red1;
    private DigitalChannel green1;
    private DigitalChannel red2;
    private DigitalChannel green2;
    private DigitalChannel red3;
    private DigitalChannel green3;

    private double servoOneInitPosition = 0.45 ;
    private double servoOneInitPositionOne = 0.7;

    private double triggermarcha1 = 1;
    private double triggermarcha2 = 1;



    // Declare OpMode members for each of the 4 motors.
    DcMotor frente_d; // Definição de dos motores em seus respectivos lados (direito e esquerdo)
    DcMotor frente_e;
    DcMotor tras_d;
    DcMotor tras_e;

    DcMotor ColetorD;
    DcMotor ColetorE;

    DcMotor corredicaD;
    DcMotor corredica;

    int andar = 0;
    int marcha = 1;
    RevColorSensorV3 sensorled;

    ElapsedTime timer = new ElapsedTime();
    Servo servo;
    Servo linear;
    Servo linear1;

    Servo servoe;
   // Servo servod;



    BNO055IMU imu;   // Declaração do Giroscópio
    Orientation angles; // Orientação da angulação do giroscópio


    @Override
    public void runOpMode() {

        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");

        red1 = hardwareMap.get(DigitalChannel.class, "red1");
        green1 = hardwareMap.get(DigitalChannel.class, "green1");

        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        green2 = hardwareMap.get(DigitalChannel.class, "green2");

        red3 = hardwareMap.get(DigitalChannel.class, "red3");
        green3 = hardwareMap.get(DigitalChannel.class, "green3");

// Mapeamento de Hardware ( Definição dos motores como "frente" e "trás")
        frente_d = hardwareMap.get(DcMotor.class, "frente_d");
        frente_e = hardwareMap.get(DcMotor.class, "frente_e");
        tras_d = hardwareMap.get(DcMotor.class, "tras_d");
        tras_e = hardwareMap.get(DcMotor.class, "tras_e");
        frente_d.setDirection(DcMotor.Direction.REVERSE);
        tras_d.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu"); // Definição do sensor de giro


        frente_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // definição de modo de parada dos motores
        frente_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tras_d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tras_e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sensorled = hardwareMap.get(RevColorSensorV3.class, "sensorled");

        servo = hardwareMap.get(Servo.class, "servo");

        linear = hardwareMap.get(Servo.class, "linear");
        linear.setDirection(Servo.Direction.FORWARD);

        linear1 = hardwareMap.get(Servo.class, "linear1");

        servoe = hardwareMap.get(Servo.class, "servoe");
     //   servod = hardwareMap.get(Servo.class, "servod");
        //servo.setDirection(Servo.Direction.REVERSE);
     //   servoe.setDirection(Servo.Direction.REVERSE);
       // servod.setDirection(Servo.Direction.REVERSE);

        ColetorE = hardwareMap.get(DcMotor.class, "coletorE");
        ColetorE.setDirection(DcMotorSimple.Direction.REVERSE);
        ColetorD = hardwareMap.get(DcMotor.class, "coletorD");

        corredica = hardwareMap.get(DcMotor.class, "slideE");
        corredica.setDirection(DcMotorSimple.Direction.FORWARD);

        corredica.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        corredica.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        corredica.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        corredica.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        corredicaD = hardwareMap.get(DcMotor.class, "slideD");
        corredicaD.setDirection(DcMotorSimple.Direction.FORWARD);

        corredicaD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        corredicaD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        corredicaD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corredicaD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

        waitForStart();

        linear.setPosition(0.9);
        linear1.setPosition(0.9);
        servo.setPosition(servoOneInitPosition);

        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        red1.setMode(DigitalChannel.Mode.OUTPUT);
        green1.setMode(DigitalChannel.Mode.OUTPUT);

        red2.setMode(DigitalChannel.Mode.OUTPUT);
        green2.setMode(DigitalChannel.Mode.OUTPUT);
        red3.setMode(DigitalChannel.Mode.OUTPUT);
        green3.setMode(DigitalChannel.Mode.OUTPUT);


        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("angles", angles);
            telemetry.addData("z", angles.firstAngle);
            telemetry.addData("y", angles.secondAngle);
            telemetry.addData("x", angles.thirdAngle);
            telemetry.addData("cor", sensorled.red());
            telemetry.update();

            //COLETOR
            if (gamepad2.left_trigger > 0) {
                ColetorE.setPower(1);
                ColetorD.setPower(1);
            } else if ((gamepad2.right_trigger > 0)) {
                ColetorE.setPower(-0.6);
                ColetorD.setPower(-0.8);
            } else {
                ColetorE.setPower(0);
                ColetorD.setPower(0);
            }

            if (gamepad2.y)
                andar = 3;
            if (gamepad2.x)
                andar = 4;
            if (gamepad2.b)
                andar = 2;

            if (gamepad2.a && andar != 1) {
                servo.setPosition(servoOneInitPositionOne);
                podeFechar = false;
                andar = 1;
            }

            if (gamepad2.start) {
                andar = 5;
            }


            telemetry.addData("Slide Speed", corredicaD.getPower());
            corredica.setPower(corredicaD.getPower());

           // slide
            switch (andar) {
                case 1:
                    PID(corredicaD, 0, timer);
                    break;
                case 2:
                    PID(corredicaD, 2000, timer);
                    break;
                case 3:
                    PID(corredicaD, 2900, timer);
                    break;
                case 4:
                    PID(corredicaD, 1200, timer);
                    break;
                case 5:
                    corredicaD.setPower(gamepad2.left_stick_y / 2);
                    break;
            }

            // LED
            if ((sensorled.red() == 85 || sensorled.blue() == 85)) {
                red.setState(true);
                green.setState(false);
                red1.setState(true);
                green1.setState(false);

                red2.setState(true);
                green2.setState(false);
                red3.setState(true);
                green3.setState(false);
            } else {
                red.setState(false);
                green.setState(true);
                red1.setState(false);
                green1.setState(true);

                red2.setState(false);
                green2.setState(true);
                red3.setState(false);
                green3.setState(true);
            }

            //agarrar cone

            if (gamepad2.dpad_right) {
                linear.setPosition(0.10);
                linear1.setPosition(0.10);
                servo.setPosition(servoOneInitPositionOne);
            } else if (gamepad2.dpad_left) {
                servo.setPosition(servoOneInitPosition);
                linear.setPosition(0.9);
                linear1.setPosition(0.9);
            }


            //SERVO
            if (gamepad2.dpad_up) {
                servoe.setPosition(1);
               // servod.setPosition(1);
            } else {
                servoe.setPosition(0);
               // servod.setPosition(0);
            }

            if (gamepad1.x)
                marcha = 1;
            if (gamepad1.y)
                marcha = 2;

            switch (marcha) {
                case 1:
                    if (gamepad1.right_trigger > 0) {
                        frente_d.setPower((gamepad1.right_trigger) / 1);
                        frente_e.setPower((-gamepad1.right_trigger) / 1);
                        tras_d.setPower((-gamepad1.right_trigger) / 1);
                        tras_e.setPower((gamepad1.right_trigger) / 1);
                    }

                    if (gamepad1.left_trigger > 0) {
                        frente_d.setPower((-gamepad1.left_trigger) / 1);
                        frente_e.setPower((gamepad1.left_trigger) / 1);
                        tras_d.setPower((gamepad1.left_trigger) / 1);
                        tras_e.setPower((-gamepad1.left_trigger) / 1);
                    }
                    if ((gamepad1.left_stick_x > -0.75) || (gamepad1.left_stick_x < 0.25)) {
                        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                            frente_e.setPower((gamepad1.left_stick_y / 1.7) + (gamepad1.left_stick_x / 2.5));
                            tras_e.setPower((gamepad1.left_stick_y / 1.7) + (gamepad1.left_stick_x / 2.5));
                            frente_d.setPower((gamepad1.left_stick_y / 1.7) - (gamepad1.left_stick_x / 2.5));
                            tras_d.setPower((gamepad1.left_stick_y / 1.7) - (gamepad1.left_stick_x / 2.5));
                        } else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) &&
                                (gamepad1.right_stick_x == 0) && (gamepad1.right_stick_y == 0)) {
                            frente_e.setPower(0);
                            tras_e.setPower(0);
                            frente_d.setPower(0);
                            tras_d.setPower(0);
                        }
                    } else if (gamepad1.left_stick_x > 0) {
                        frente_e.setPower((-1) / 1.7);
                        tras_e.setPower((-1) / 1.7);
                        frente_d.setPower((1) / 1.7);
                        tras_d.setPower((1) / 1.7);
                    } else {
                        frente_e.setPower((1) / 1.7);
                        tras_e.setPower((1) / 1.7);
                        frente_d.setPower((-1) / 1.7);
                        tras_d.setPower((-1) / 1.7);
                    }

                    if ((gamepad1.right_stick_x > -0.75) || (gamepad1.right_stick_x < 0.25)) {
                        if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                            frente_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
                            tras_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
                            frente_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
                            tras_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
                        }
                    } else if (gamepad1.right_stick_x > 0) {
                        frente_e.setPower(-0.2);
                        tras_e.setPower(-0.2);
                        frente_d.setPower(0.2);
                        tras_d.setPower(0.2);
                    } else {
                        frente_e.setPower(0.2);
                        tras_e.setPower(0.2);
                        frente_d.setPower(-0.2);
                        tras_d.setPower(-0.2);
                    }
                    break;


                case 2:
                    if (gamepad1.right_trigger > 0) {
                        frente_d.setPower((gamepad1.right_trigger));
                        frente_e.setPower((-gamepad1.right_trigger));
                        tras_d.setPower((-gamepad1.right_trigger));
                        tras_e.setPower((gamepad1.right_trigger));
                    }

                    if (gamepad1.left_trigger > 0) {
                        frente_d.setPower((-gamepad1.left_trigger));
                        frente_e.setPower((gamepad1.left_trigger));
                        tras_d.setPower((gamepad1.left_trigger));
                        tras_e.setPower((-gamepad1.left_trigger));
                    }
                    if ((gamepad1.left_stick_x > -0.75) || (gamepad1.left_stick_x < 0.25)) {
                        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                            frente_e.setPower((gamepad1.left_stick_y) + (gamepad1.left_stick_x));
                            tras_e.setPower((gamepad1.left_stick_y) + (gamepad1.left_stick_x));
                            frente_d.setPower((gamepad1.left_stick_y) - (gamepad1.left_stick_x));
                            tras_d.setPower((gamepad1.left_stick_y) - (gamepad1.left_stick_x));
                        } else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) &&
                                (gamepad1.right_stick_x == 0) && (gamepad1.right_stick_y == 0)) {
                            frente_e.setPower(0);
                            tras_e.setPower(0);
                            frente_d.setPower(0);
                            tras_d.setPower(0);
                        }
                    } else if (gamepad1.left_stick_x > 0) {
                        frente_e.setPower((-1));
                        tras_e.setPower((-1));
                        frente_d.setPower((1));
                        tras_d.setPower((1));
                    } else {
                        frente_e.setPower((1));
                        tras_e.setPower((1));
                        frente_d.setPower((-1));
                        tras_d.setPower((-1));
                    }

                    if ((gamepad1.right_stick_x > -0.75) || (gamepad1.right_stick_x < 0.25)) {
                        if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                            frente_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
                            tras_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
                            frente_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
                            tras_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
                        }
                    } else if (gamepad1.right_stick_x > 0) {
                        frente_e.setPower(-0.2);
                        tras_e.setPower(-0.2);
                        frente_d.setPower(0.2);
                        tras_d.setPower(0.2);
                    } else {
                        frente_e.setPower(0.2);
                        tras_e.setPower(0.2);
                        frente_d.setPower(-0.2);
                        tras_d.setPower(-0.2);
                    }
                    break;
            }

//            //MOVIMENTAÇÃO DO ROBÔ
//            if (gamepad1.right_trigger > 0) {
//                frente_d.setPower((gamepad1.right_trigger) / 1);
//                frente_e.setPower((-gamepad1.right_trigger) / 1);
//                tras_d.setPower((-gamepad1.right_trigger) / 1);
//                tras_e.setPower((gamepad1.right_trigger) / 1);
//            }
//
//            if (gamepad1.left_trigger > 0) {
//                frente_d.setPower((-gamepad1.left_trigger) / 1);
//                frente_e.setPower((gamepad1.left_trigger) / 1);
//                tras_d.setPower((gamepad1.left_trigger) / 1);
//                tras_e.setPower((-gamepad1.left_trigger) / 1);
//            }
//
//            if ((gamepad1.left_stick_x > -0.75) || (gamepad1.left_stick_x < 0.25)) {
//                if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
//                    frente_e.setPower((gamepad1.left_stick_y / 1.5) + (gamepad1.left_stick_x / 2.3));
//                    tras_e.setPower((gamepad1.left_stick_y / 1.5) + (gamepad1.left_stick_x / 2.3));
//                    frente_d.setPower((gamepad1.left_stick_y / 1.5) - (gamepad1.left_stick_x / 2.3));
//                    tras_d.setPower((gamepad1.left_stick_y / 1.5) - (gamepad1.left_stick_x / 2.3));
//                } else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) &&
//                        (gamepad1.right_stick_x == 0) && (gamepad1.right_stick_y == 0)) {
//                    frente_e.setPower(0);
//                    tras_e.setPower(0);
//                    frente_d.setPower(0);
//                    tras_d.setPower(0);
//                }
//            } else if (gamepad1.left_stick_x > 0) {
//                frente_e.setPower((-1) / 1.5);
//                tras_e.setPower((-1) / 1.5);
//                frente_d.setPower((1) / 1.5);
//                tras_d.setPower((1) / 1.5);
//            } else {
//                frente_e.setPower((1) / 1.5);
//                tras_e.setPower((1) / 1.5);
//                frente_d.setPower((-1) / 1.5);
//                tras_d.setPower((-1) / 1.5);
//            }
//
//            if ((gamepad1.right_stick_x > -0.75) || (gamepad1.right_stick_x < 0.25)) {
//                if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
//                    frente_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
//                    tras_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
//                    frente_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
//                    tras_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
//                }
//            } else if (gamepad1.right_stick_x > 0) {
//                frente_e.setPower(-0.2);
//                tras_e.setPower(-0.2);
//                frente_d.setPower(0.2);
//                tras_d.setPower(0.2);
//            } else {
//                frente_e.setPower(0.2);
//                tras_e.setPower(0.2);
//                frente_d.setPower(-0.2);
//                tras_d.setPower(-0.2);
//            }

        }

    }

    boolean podeFechar = true;

    public void PID(DcMotor motor, int reference, ElapsedTime timer) {
        double Kp = 0.0013; //6
        double Ki = 0.011; //3
        double Kd = 0;
        double lastError = 0;
        double integralSum = 0;

        double encoderPosition = motor.getCurrentPosition();
        double error = reference - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        double limit = 0.8;

        if (out > limit) {
            out = limit;
        } else if (out < -limit) {
            out = -limit;
        }

        lastError = error;

        timer.reset();

        telemetry.addData("Error", error);
        telemetry.addData("Output", out);
        telemetry.addData("Pos", motor.getCurrentPosition());
        telemetry.addData("Level", andar);

        if (motor.getCurrentPosition() < 100 && andar == 1 && !podeFechar) {
            servo.setPosition(servoOneInitPosition);
        }

        if (motor.getCurrentPosition() > (reference - 5) && motor.getCurrentPosition() < (reference + 5)) {
            motor.setPower(0);

            if (gamepad2.dpad_right) {
                servo.setPosition(servoOneInitPositionOne);
                podeFechar = true;
            }

        } else {
            motor.setPower(out);
        }
    }
}