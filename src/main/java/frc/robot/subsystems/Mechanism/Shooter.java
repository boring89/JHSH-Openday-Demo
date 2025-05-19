package frc.robot.subsystems.Mechanism;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final SparkMax ArmMotor, ShooterMotor;
    private final RelativeEncoder ArmEncoder;
    private final PIDController ArmController;
    private final Elevator elevator;
    private final AnalogInput IRsensor;
    private boolean shoot = false;

    public Shooter(Elevator elevator) {
        this.elevator = elevator;
        ArmMotor = new SparkMax(15, MotorType.kBrushless);
        ArmEncoder = ArmMotor.getEncoder();
        ShooterMotor = new SparkMax(16, MotorType.kBrushless);
        ArmController = new PIDController(0.2, 0, 0);
        IRsensor = new AnalogInput(7);
    }

    public double getPosition() {
        return ArmEncoder.getPosition();
    }

    public void setPosition() {
        if (elevator.getPosition() > 50) {
            ArmMotor.set(ArmController.calculate(getPosition(), 26));
        }else {
            ArmMotor.set(ArmController.calculate(getPosition(), 0));
        }
    }

    public void Mode() {
        if (elevator.getPosition() < 10) {
            if (IRsensor.getVoltage() < 10) {
                ShooterMotor.set(0.2);
            }else {
                ShooterMotor.set(0);
            }
        }else {
            if (shoot) {
                ShooterMotor.set(1);
            }else {
                ShooterMotor.set(0);
            }
        }
    }

    public void shoot() {
        shoot = true;
    }

    public void stop() {
        shoot = false;
    }

    @Override
    public void periodic() {
        setPosition();
        Mode();
    }
}
