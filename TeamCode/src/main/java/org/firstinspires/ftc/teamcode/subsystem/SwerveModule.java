package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static kotlin.text.Typography.tm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class SwerveModule {
    public static double P = 0.0, I = 0.0, D = 0.0;
    public static double K_STATIC = 0.0;

    public static double MAX_MOTOR = 1;

    public static boolean MOTOR_FLIPPING = true;

    public static double WHEEL_RADIUS = 1.42;//in
    public static double GEAR_RATIO = 1 / (2 * 4.5);
    public static final double TICKS_PER_REV = 28;

    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private PIDFController rotationController;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double position = 0.0;

    public SwerveModule(DcMotorEx tm, DcMotorEx bm) {
        topMotor = tm;
        MotorConfigurationType topMotorConfigurationType = topMotor.getMotorType().clone();
        topMotorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        topMotor.setMotorType(topMotorConfigurationType);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomMotor = bm;
        MotorConfigurationType bottomMotorConfigurationType = bottomMotor.getMotorType().clone();
        bottomMotorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        bottomMotor.setMotorType(bottomMotorConfigurationType);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotationController = new PIDFController(P, I, D, 0);
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public SwerveModule(HardwareMap hardwareMap, String tmName, String bmName) {
        this(hardwareMap.get(DcMotorEx.class, tmName),
                hardwareMap.get(DcMotorEx.class, bmName));
    }

    public void read() {
        position = Math.toRadians((topMotor.getCurrentPosition()+bottomMotor.getCurrentPosition() ) / TICKS_PER_REV * GEAR_RATIO) - Math.PI;
    }

    public void update() {
        rotationController.setPIDF(P, I, D, 0);
        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);
        if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = normalizeRadians(target - current);

        double power = Range.clip(rotationController.calculate(0, error), -MAX_MOTOR, MAX_MOTOR);
        if (Double.isNaN(power)) power = 0;
        double rotationalPower = power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power);
    }

    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(position - Math.PI);
    }

}
