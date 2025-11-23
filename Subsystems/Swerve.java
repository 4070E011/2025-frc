package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.SPI;   // 引入 SPI 介面
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constanats.CANDevicesID;
import frc.robot.Constanats.RobotConstants;
import frc.robot.Modules.SwerveModule;

public class Swerve extends SubsystemBase {

  // ... (原本的 SwerveModule 宣告保持不變) ...
  private SwerveModule LeftFront, RightFront, RightRear, LeftRear;
  
  private SwerveDriveKinematics kinematics;

  private SwerveDriveOdometry odometry;
  private final Field2d field = new Field2d();
  
  // 【新增】宣告 navX 陀螺儀
  private AHRS gyro;

  public Swerve() {
    // 【新增】初始化陀螺儀 (通常插在 RoboRIO 中間的 MXP Port)
    try {
        gyro = new AHRS(NavXComType.kMXP_SPI); 
    } catch (RuntimeException ex) {
        System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    // ... (原本的 SwerveModule 初始化程式碼保持不變) ...
    LeftFront = new SwerveModule(CANDevicesID.kLFDriveMotorID, CANDevicesID.kLFTurnMotorID, CANDevicesID.kLFCancoderID);
    RightFront = new SwerveModule(CANDevicesID.kRFDriveMotorID, CANDevicesID.kRFTurnMotorID, CANDevicesID.kRFCancoderID);
    RightRear = new SwerveModule(CANDevicesID.kRRDriveMotorID, CANDevicesID.kRRTurnMotorID, CANDevicesID.kRRCancoderID);
    LeftRear = new SwerveModule(CANDevicesID.kLRDriveMotorID, CANDevicesID.kLRTurnMotorID, CANDevicesID.kLRCancoderID);

    // ... (原本的 Config 設定保持不變) ...
    
    // 建議在啟動時稍微等一下再歸零，確保陀螺儀穩定 (非必須，但推薦)
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {}
    }).start();
    
    kinematics = new SwerveDriveKinematics(
      new Translation2d(0.5, 0.5),
      new Translation2d(0.5, -0.5),
      new Translation2d(-0.5, -0.5),
      new Translation2d(-0.5, 0.5)
    );

    odometry = new SwerveDriveOdometry(
      kinematics, 
      getHeading(), 
      new SwerveModulePosition[] {
        LeftFront.getPosition(),
        RightFront.getPosition(),
        RightRear.getPosition(),
        LeftRear.getPosition()
      }
    );

    SmartDashboard.putData("Field", field);
  }

  // 【新增】陀螺儀歸零 (通常綁定在按鈕上)
  public void zeroHeading() {
    gyro.reset();
  }

  // 【新增】取得機器人當前角度 (Rotation2d 格式)
  // navX 預設是順時針為正，WPILib 需要逆時針為正，所以通常要加負號或使用 getRotation2d()
  public Rotation2d getHeading() {
    // getRotation2d() 在新版 navX 函式庫通常已經處理好座標系，如果發現旋轉反向，請改用:
    // return Rotation2d.fromDegrees(-gyro.getAngle());
    return gyro.getRotation2d();
  }

  // 修改 drive 介面，加入 fieldRelative 參數
  public void drive(double x, double y, double turn, boolean fieldRelative) {
    
    // 1. 判斷是「場地導向」還是「機器人導向」
    ChassisSpeeds speeds;
    
    if (fieldRelative) {
        // 【重點】場地導向：需要傳入目前的陀螺儀角度 (getHeading())
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x * RobotConstants.kMaxSpeed, 
            y * RobotConstants.kMaxSpeed, 
            turn * RobotConstants.kMaxAngularSpeed,
            getHeading()
        );
    } else {
        // 機器人導向 (傳統模式)
        speeds = new ChassisSpeeds(
            x * RobotConstants.kMaxSpeed, 
            y * RobotConstants.kMaxSpeed, 
            turn * RobotConstants.kMaxAngularSpeed
        );
    }

    // 2. 轉換成模組狀態
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    // 3. 速度正規化
    SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConstants.kMaxSpeed);
    
    // 4. 分配狀態
    LeftFront.setState(states[0]);
    RightFront.setState(states[1]);
    RightRear.setState(states[2]);
    LeftRear.setState(states[3]);
  }

  @Override
  public void periodic() {
    odometry.update(
      getHeading(), 
      new SwerveModulePosition[] {
        LeftFront.getPosition(),
        RightFront.getPosition(),
        RightRear.getPosition(),
        LeftRear.getPosition()
      }
    );

    field.setRobotPose(odometry.getPoseMeters());
  }
}