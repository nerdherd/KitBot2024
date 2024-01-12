package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.preferences.PrefDouble;

public class Shooter extends SubsystemBase {
    TalonFX topMotor;
    TalonFX bottomMotor;

    private double[] speeds = {-0.1, 0.5, 0.7};
    private double[] speeds2 = {-0.1, 0.5, 0.7};
    private int index = 0;

    // public double topVelocity;
    // public double bottomVelocity;
    // PIDController shooterController;

    // public PrefDouble ktopMotorP = new PrefDouble("Top Motor kP", 0);
    // public PrefDouble kbottomMotorP = new PrefDouble("Bottom Motor kP", 0);
    
    // public PrefDouble ktopMotorF = new PrefDouble("Top Motor kF", 0);
    // public PrefDouble kbottomMotorF = new PrefDouble("Bottom Motor kF", 0);


    public Shooter() {    
        topMotor = new TalonFX(ShooterConstants.kTopMotorID);
        bottomMotor = new TalonFX(ShooterConstants.kBottomMotorID);

        init();
    }
    
    public void init() {
        topMotor.setInverted(false);
        // bottomMotor.setInverted(false);
        bottomMotor.setInverted(TalonFXInvertType.FollowMaster);

        topMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.setNeutralMode(NeutralMode.Brake);

        // ShooterConstants.kPTopMotor.loadPreferences();
        // ShooterConstants.kITopMotor.loadPreferences();
        // ShooterConstants.kDTopMotor.loadPreferences();
        // ShooterConstants.kFTopMotor.loadPreferences();

        // ShooterConstants.kPBottomMotor.loadPreferences();
        // ShooterConstants.kIBottomMotor.loadPreferences();
        // ShooterConstants.kDBottomMotor.loadPreferences();
        // ShooterConstants.kFBottomMotor.loadPreferences();
        
        // topMotor.config_kP(0,ShooterConstants.kPTopMotor.get());
        // topMotor.config_kF(0,ShooterConstants.kFTopMotor.get());
        // bottomMotor.config_kP(0,ShooterConstants.kPBottomMotor.get());
        // bottomMotor.config_kF(0,ShooterConstants.kFBottomMotor.get());

        // topMotor.configNeutralDeadband(0.05);
        // bottomMotor.configNeutralDeadband(0.05);
        topMotor.configVoltageCompSaturation(11, 0);
        bottomMotor.configVoltageCompSaturation(11, 0);
        topMotor.enableVoltageCompensation(true);
        bottomMotor.enableVoltageCompensation(true);

    }
    
    public void printSpeeds() {
        SmartDashboard.putNumber("Index", index);
        SmartDashboard.putNumber("IntakeTop", speeds[0]);
        SmartDashboard.putNumber("OuttakeTop1", speeds[1]);
        SmartDashboard.putNumber("OuttakeTop2", speeds[2]);
        SmartDashboard.putNumber("IntakeBottom", speeds2[0]);
        SmartDashboard.putNumber("OuttakeBottom1", speeds2[1]);
        SmartDashboard.putNumber("OuttakeBottom2", speeds2[2]);
    }

    public CommandBase setIndex(int index) {
        return Commands.runOnce(() -> {
            this.index = index;
        });
    }

    public CommandBase setSpeed() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, speeds[index]);
            bottomMotor.set(ControlMode.PercentOutput, speeds2[index]);
        });
    }

    public CommandBase setPowerZero() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, 0);
            bottomMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public CommandBase increaseTop() {
        return Commands.runOnce(() -> {
            this.speeds[index] += 0.1;
        });
    }
    public CommandBase increaseBottom() {
        return Commands.runOnce(() -> {
            this.speeds2[index] += 0.1;
        });
    }

    public CommandBase decreaseTop() {
        return Commands.runOnce(() -> {
            this.speeds[index] -= 0.1;
        });
    }
    public CommandBase decreaseBottom() {
        return Commands.runOnce(() -> {
            this.speeds2[index] -= 0.1;
        });
    }


    // public CommandBase setVelocity(double topVelocity, double bottomVelocity){
    //     return runOnce(
    //         () -> {
    //             topMotor.set(ControlMode.Velocity, topVelocity);
    //             bottomMotor.set(ControlMode.Velocity, bottomVelocity);
    //         }
    //     );
    // }

    // public CommandBase setVelocityZero() {
    //     return Commands.runOnce(
    //         () -> setVelocity(0,0)
    //     );
    // }

    // public CommandBase intake(double topVelocity, double bottomVelocity) {
    //     return Commands.sequence(
    //         setVelocity(topVelocity, bottomVelocity),
    //         Commands.waitSeconds(1),
    //         setVelocityZero()
    //         );
    // }

    // public CommandBase outtake(double topVelocity, double bottomVelocity) {
    //     return Commands.sequence(
    //         setVelocity(topVelocity, bottomVelocity),
    //         Commands.waitSeconds(1),
    //         setVelocityZero()
    //     );
    // }

    public void reportToSmartDashboard(){
        // leftShooter.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Power", 0));
        SmartDashboard.putNumber("Left RPM", topMotor.getSelectedSensorVelocity(0) * 10 / 2048);
        SmartDashboard.putNumber("Right RPM", bottomMotor.getSelectedSensorVelocity(0) * 10 / 2048);
        SmartDashboard.putNumber("Left Current", topMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Right Current", bottomMotor.getSupplyCurrent());
        // SmartDashboard.putNumber("Percent Output Bottom", percentOutputBottom);
        // SmartDashboard.putNumber("Percent Output Top", percentOutputTop);
    }

    // public void initShuffleobard() {
    //     ShuffleboardTab tab = Shuffleboard.getTab("Flywheel");

    //     tab.addNumber("Top Motor Velocity", topMotor::getSelectedSensorVelocity);
    //     tab.addNumber("Bottom Motor Velocity", bottomMotor::getSelectedSensorVelocity);
    //     tab.addNumber("Top Motor Voltage", topMotor::getMotorOutputVoltage);
    //     tab.addNumber("Bottom Motor Voltage", bottomMotor::getMotorOutputVoltage);
    //     tab.addNumber("Top Motor Output Percent", topMotor::getMotorOutputPercent);
    //     tab.addNumber("Bottom Motor Output Percent", bottomMotor::getMotorOutputPercent);
        // tab.addNumber("Top Motor Target Velocity", () -> topVelocity);
        // tab.addNumber("Top Motor Target Velocity", () -> bottomVelocity);

    // }

}
