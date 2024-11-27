package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class servosConfig{
        public Servo servo = null;

        public Servo servo_2 = null;

        public static final double MIN_SERVO = 0;
        public static final double MID_SERVO = 0.5;
        public static final double MAX_SERVO = 1;

        HardwareMap hwMap = null;

        public void init(HardwareMap ahwMap, Telemetry telemetry) {

            hwMap = ahwMap;

            servo = hwMap.get(Servo.class, "servo");
            servo_2 = hwMap.get(Servo.class,  "servo_2");
            servo.setPosition(MID_SERVO);
            servo.setPosition(MID_SERVO);

        }

}
