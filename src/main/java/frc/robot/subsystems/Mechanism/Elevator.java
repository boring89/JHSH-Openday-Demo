package frc.robot.subsystems.Mechanism;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final TalonFX LMotor, RMotor;
    private final PIDController ElevatorController;
    private int level = 0;

    public Elevator() {

        LMotor = new TalonFX(13);
        RMotor = new TalonFX(14);
        RMotor.setControl(new Follower(13, false));

        ElevatorController = new PIDController(0.2, 0, 0);
    }

    public double getPosition() {
        return LMotor.getPosition().getValueAsDouble();
    }

    public void setPosition() {
        if (level == 0) {
            LMotor.set(ElevatorController.calculate(getPosition(), 0.0));
        }else {
            LMotor.set(ElevatorController.calculate(getPosition(), 93.5));
        }
    }

    public void changeLevel() {
        if (level == 0) {
            level = 1;
        }else {
            level = 0;
        }
    }

    @Override
    public void periodic() {
        setPosition();
    }
    
}
