package frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable Limelight;
    private final NetworkTableEntry x, a;
    private double[] pose;
    private final PIDController xController, yController, rotController;
    private double xOut, yOut, rotOut;

    public Limelight() {
        Limelight = NetworkTableInstance.getDefault().getTable("limelight");
        x = Limelight.getEntry("tx");
        a = Limelight.getEntry("ta");
        xController = new PIDController(0.1, 0, 0);
        xController.setTolerance(0.5);
        xController.setIntegratorRange(-0.5, 0.5);
        yController = new PIDController(0.1, 0, 0);
        yController.setTolerance(0.5);
        yController.setIntegratorRange(-0.5, 0.5);
        rotController = new PIDController(0.05, 0, 0);
        rotController.setTolerance(0.3);
        rotController.setIntegratorRange(-0.3, 0.3);
    }

    public double getX() {
        return x.getDouble(0);
    }

    public double getA() {
        return a.getDouble(0);
    }

    public double getPose() {
        if (pose != null && pose.length >= 5) {
            return pose[4];
        }else {
            System.out.println("No value!!!");
            return 0;
        }
    }

    public double xOut() {
        return xOut;
    }

    public double yOut() {
        return yOut;
    }

    public double rotOut() {
        return rotOut;
    }

    @Override 
    public void periodic() {
        xOut = Math.abs(getX()) > 0.1
            ? MathUtil.clamp(xController.calculate(getX()), -0.5, 0.5)
            : 0;
        yOut = Math.abs(getA() - 5) > 0.1
            ? MathUtil.clamp(yController.calculate(getA(), 5), -0.5, 0.5)
            : 0;
        rotOut = Math.abs(getPose()) > 0.1
            ? MathUtil.clamp(rotController.calculate(getPose()), -0.2, 0.2)
            : 0;

        pose = Limelight.getEntry("botpose_targetSpace").getDoubleArray(new double[6]);   
    }
}
