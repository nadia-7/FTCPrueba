package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PruebaServos", group="Pushbot")
public class servos extends LinearOpMode {

    servosConfig servos = new servosConfig();

    @Override
    public void runOpMode() {
        servos.init(hardwareMap , telemetry);
        sleep(1000);
        telemetry.update();

        waitForStart();

        double posicion = servos.servo.getPosition();
        double posicion_2 = servos.servo_2.getPosition();

        while (opModeIsActive()) {
            telemetry.addLine("Cambia la posicion del servo con los bumpers");

            telemetry.addData("servo 1" , posicion);
            telemetry.addData("servo2", posicion_2);
            telemetry.update();



            if (gamepad1.a) {
                posicion += 0.05;
                posicion_2 -= 0.05;
                sleep(300);
            }
            else if (gamepad1.b) {
                posicion -= 0.05;
                posicion_2 += 0.05;
                sleep(300);
            }
            servos.servo.setPosition(posicion);
            servos.servo_2.setPosition(posicion_2);

        }
    }
}
