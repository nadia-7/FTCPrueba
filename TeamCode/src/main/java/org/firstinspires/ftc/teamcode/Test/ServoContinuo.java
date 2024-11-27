package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//asegurate de programar el servo en modo continuo

@TeleOp(name="ServoContinuo", group="Pushbot")

    public class ServoContinuo extends LinearOpMode {
//.8

        CRServo servo;
        //HardwareMap hwMap = null;

        public void init(HardwareMap hwmap, Telemetry telemetry){
            servo = hwmap.get(CRServo.class, "servo");
           // servo.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        @Override
        public void runOpMode() {
            init(hardwareMap, telemetry);
            sleep(1000);
            telemetry.update();
            servo.setPower(0);
            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Posicion del servo" , servo.getPower());
                telemetry.update();

                while (gamepad1.a) {
                    servo.setPower(1);
                }
                while (gamepad1.b) {
                    servo.setPower(-1);
                }  servo.setPower(0);
            }
        }
    }
