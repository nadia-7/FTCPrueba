package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class actions {

    //==Declarar Webadas Rancias==
    public DcMotor elevador1 = null;
    public DcMotor elevador2 = null;
    public Servo CorrederaGarra;
    public Servo CorrederaGarra2;
    public Servo servo_Brazo1;
    public Servo servo_Brazo2;
    public Servo ArticulacionGarra;
    public Servo servo_Garra;

    //==Funciones de Movimiento==
    public void elevadorEnfrente(double POWER){
        elevador1.setPower(POWER);
        elevador2.setPower(POWER);
    }
    public void elevadorAtras(double POWER){
        elevador1.setPower(-POWER);
        elevador2.setPower(-POWER);
    }
    public void moverCorredera(double POS, double POS2){
        CorrederaGarra.setPosition(POS); // aumenta
        CorrederaGarra2.setPosition(POS2); // disminuye
    }
    public void moverBrazo(double POS1, double POS2) {
        servo_Brazo1.setPosition(POS1);
        servo_Brazo2.setPosition(POS2);
    }
    public void moverAGarra(double POS){
        ArticulacionGarra.setPosition(POS);
    }

    //==Funciones de Iniciacion==
    public void initElevador(){
        elevador1 = hardwareMap.get(DcMotor.class,"elevador1");
        elevador2 = hardwareMap.get(DcMotor.class,"elevador2");
//        elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevador2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevador2.setDirection(DcMotorSimple.Direction.REVERSE);
        elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Elevadores iniciados");
    }
    public void initGarra(){
        CorrederaGarra = hardwareMap.get(Servo.class, "Corredera1");
        CorrederaGarra2 = hardwareMap.get(Servo.class, "Corredera2");
        servo_Brazo1 = hardwareMap.get(Servo.class, "brazo1");
        servo_Brazo2 = hardwareMap.get(Servo.class, "brazo2");
        ArticulacionGarra = hardwareMap.get(Servo.class, "hand");
        servo_Garra = hardwareMap.get(Servo.class, "garra");
        ArticulacionGarra.setPosition(0.5);
        telemetry.addLine("Garra iniciada");
        telemetry.addLine("Garra iniciada");
    }


}
