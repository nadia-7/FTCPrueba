package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="PruebaServo", group="Pushbot")

public class ServoTest extends LinearOpMode {
//.8

    Servo servo = null;
    //HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap, Telemetry telemetry){
        servo = ahwMap.get(Servo.class, "servo");
        servo.setPosition(.4);
    }

    @Override
    public void runOpMode() {
        init(hardwareMap, telemetry);
        sleep(1000);
        telemetry.update();

        waitForStart();

        double posicion = .75;

        while (opModeIsActive()) {
            telemetry.addLine("Cambia la posicion del servo con los bumpers");

            telemetry.addData("" , "");
            telemetry.addData("Posicion del servo" , posicion);
            telemetry.update();



            if (gamepad1.a && posicion < .75) {
                posicion += .35;
                sleep(300);
            }
            else if (gamepad1.b && posicion > .4) {
                posicion -= .35;
                sleep(300);
            }

            servo.setPosition(posicion);


        }
    }
}

