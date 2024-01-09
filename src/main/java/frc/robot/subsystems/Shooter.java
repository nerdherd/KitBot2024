package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.preferences.PrefDouble;

public class Shooter extends SubsystemBase {
    TalonFX topMotor;
    TalonFX bottomMotor;
    public double topVelocity;
    public double bottomVelocity;
    PIDController shooterController;

    public PrefDouble ktopMotorP = new PrefDouble("Top Motor kP", 0);
    public PrefDouble kbottomMotorP = new PrefDouble("Bottom Motor kP", 0);
    
    public PrefDouble ktopMotorF = new PrefDouble("Top Motor kF", 0);
    public PrefDouble kbottomMotorF = new PrefDouble("Bottom Motor kF", 0);


    public Shooter() {    
        topMotor = new TalonFX(ShooterConstants.kTopMotorID);
        bottomMotor = new TalonFX(ShooterConstants.kBottomMotorID);

        init();
    }
    
    public void init() {
        topMotor.setInverted(false);
        bottomMotor.setInverted(false);

        topMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.setNeutralMode(NeutralMode.Brake);

        ShooterConstants.kPTopMotor.loadPreferences();
        ShooterConstants.kITopMotor.loadPreferences();
        ShooterConstants.kDTopMotor.loadPreferences();
        ShooterConstants.kFTopMotor.loadPreferences();

        ShooterConstants.kPBottomMotor.loadPreferences();
        ShooterConstants.kIBottomMotor.loadPreferences();
        ShooterConstants.kDBottomMotor.loadPreferences();
        ShooterConstants.kFBottomMotor.loadPreferences();

        topMotor.config_kP(0,ShooterConstants.kPTopMotor.get());
        topMotor.config_kF(0,ShooterConstants.kFTopMotor.get());
        bottomMotor.config_kP(0,ShooterConstants.kPBottomMotor.get());
        bottomMotor.config_kF(0,ShooterConstants.kFBottomMotor.get());

        topMotor.configNeutralDeadband(0.05);
        bottomMotor.configNeutralDeadband(0.05);
        topMotor.configVoltageCompSaturation(11, 0);
        bottomMotor.configVoltageCompSaturation(11, 0);
        topMotor.enableVoltageCompensation(true);
        bottomMotor.enableVoltageCompensation(true);

    }
    public CommandBase setVelocity(double topVelocity, double bottomVelocity){
        return runOnce(
            () -> {
                topMotor.set(ControlMode.Velocity, topVelocity);
                bottomMotor.set(ControlMode.Velocity, bottomVelocity);
            }
        );
    }

    public CommandBase setVelocityZero() {
        return Commands.runOnce(
            () -> setVelocity(0,0)
        );
    }

    public CommandBase intake(double topVelocity, double bottomVelocity) {
        return Commands.sequence(
            setVelocity(topVelocity, bottomVelocity),
            Commands.waitSeconds(1),
            setVelocityZero()
            );
    }

    public CommandBase outtake(double topVelocity, double bottomVelocity) {
        return Commands.sequence(
            setVelocity(topVelocity, bottomVelocity),
            Commands.waitSeconds(1),
            setVelocityZero()
        );
    }


    public void initShuffleobard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Flywheel");

        tab.addNumber("Top Motor Velocity", topMotor::getSelectedSensorVelocity);
        tab.addNumber("Bottom Motor Velocity", bottomMotor::getSelectedSensorVelocity);
        tab.addNumber("Top Motor Voltage", topMotor::getMotorOutputVoltage);
        tab.addNumber("Bottom Motor Voltage", bottomMotor::getMotorOutputVoltage);
        tab.addNumber("Top Motor Output Percent", topMotor::getMotorOutputPercent);
        tab.addNumber("Bottom Motor Output Percent", bottomMotor::getMotorOutputPercent);
        tab.addNumber("Top Motor Target Velocity", () -> topVelocity);
        tab.addNumber("Top Motor Target Velocity", () -> bottomVelocity);

    }

}
