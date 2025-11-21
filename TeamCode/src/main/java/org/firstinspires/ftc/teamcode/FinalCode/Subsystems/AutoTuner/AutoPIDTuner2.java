//package org.firstinspires.ftc.teamcode.Experiments.Subsystems.AutoTuner;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.BCPath;
//import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.GVF.VectorField;
//import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.Odometry;
//import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Drivetrain.WheelControl;
//import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
//import org.opencv.core.Point;
//
//import java.lang.reflect.Array;
//import java.util.ArrayList;
//import java.util.List;
//import java.util.TreeMap;
//
//@TeleOp
//@Config
//public class AutoPIDTuner2 extends OpMode{
//    int n = 3;
//    List<PIDFController> controllers = new ArrayList<>(n);
//    public static TreeMap<Double, Vec3d> errors;
//
//    public static boolean uk = false;
//
//    Odometry odometry;
//    WheelControl wheelControl;
//    ZieglerNichols tuner;
//    Vec3d target = new Vec3d(100, 100, 90);
//
//    enum State{
//        runSystem,
//        returnOriginalSystem,
//        autoTune,
//    }
//
//    State state = State.runSystem;
//
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    @Override
//    public void init() {
//        odometry = new Odometry(hardwareMap, telemetry, 8, 8, 0);
//        wheelControl = new WheelControl(hardwareMap, odometry);
//        tuner = new ZieglerNichols();
//
//        for (int i=0; i<n; i++){
//            controllers.add(new PIDFController(0.2, 0, 0, 0));
//        }
//    }
//    @Override
//    public void loop(){
//        odometry.update();
//        Vec3d pos = new Vec3d(odometry.get_x(uk), odometry.get_y(uk), odometry.get_heading(uk));
//
//        switch (state){
//            case runSystem:
//                for (int i=0; i<n; i++){
//                    controllers.get(i).calculate(target.get(i), pos.get(i));
//                    wheelControl.drive();
//                }
//        }
//
//    }
//
//}
