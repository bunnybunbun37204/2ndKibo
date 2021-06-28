package jp.jaxa.iss.kibo.rpc.sampleapk;

//android lib
import android.graphics.Bitmap;
import android.util.Log;

//Zxing lib
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

//Nasa lib
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;


//openCV lib

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.aruco.*;
import  org.opencv.core.Scalar;
import org.opencv.calib3d.Calib3d;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;
import static org.opencv.android.Utils.matToBitmap;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class YourService extends KiboRpcService {


    public int pattern;
    public float px, py, pz;
    final int NAV_MAX_COL = 1280;
    final int NAV_MAX_ROW =  960;
    private imgProcessing imgProc;
    private quaternion orientation;
    private static camera_params camparams;
    private static Mat camMatrix;
    private static Mat dstMatrix;
    private static int dictID = Aruco.DICT_5X5_250;
    private static Mat ids;
    private static ArrayList<Mat> corners;
    private static Dictionary dict;
    private static Scalar borderColor;
    private static float markerSize;
    public double tax,tay,taz;
    private static Mat rvecs;
    private static Mat tvecs;
    private static Mat mean_rvecs;
    private  Quaternion q;
    private double tabx,taby,tabz;

    public static void setCamCalibration() {
        camparams = new camera_params();
        camparams.process_camera();
        Log.d("LOG-DEBUGGER","FINISH SET CAMPARAMS.PROCESS");
        camMatrix=camparams.getCamMatrix();
        dstMatrix=camparams.getDistortionMat();
        Log.i("LOG-INFO :","camMatrix : "+camMatrix+" dstMatrix "+dstMatrix);
    }

    @Override
    protected void runPlan1() {
        // astrobee is undocked and the mission starts
//        try{
//            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//        }
//        catch (Exception e){
//            String err = e.getMessage();
//            Log.d("LOG-DEBUGGER","ERR IS : "+err);
//        }
//        Log.d("LOG-DEBUGGER","FINISH LOAD NATIVE LIB");
        Log.d("LOG-DEBUGGER","NATIVE LIBRARY "+Core.NATIVE_LIBRARY_NAME);
        setCamCalibration();
        Log.d("LOG-DEBUGGER","FINISH SETCAM");
        readQRImageDECOY();
        Log.d("LOG-DEBUGGER","FINISH IMPLEMENTATION");
        api.startMission();
        //moveToWrapper(11.21,-9.8,4.79, 0, 0, -0.707, 0.707);
        moveToWrapper(11.35, -9.9, 5, 0, 0, -0.707, 0.707);
        //Log.i("LOG-INFP","A : "+api.getImu().getLinearAccelerationCovariance().toString());
//        Log.i("LOG-INFO","KINEMATIC POSITION : "+api.getRobotKinematics().getPosition().toString());
//        Log.i("LOG-INFO","TRUSTED KINEMETIC POSITION : "+api.getTrustedRobotKinematics().getPosition().toString());
        Log.d("LOG-DEBUGGER","TO POINT-A");
        //Quaternion quaternion = eulertToQuaternion(0,0,0);
        //Log.d("LOG-DEBUGGER","qx is "+quaternion.getX()+" qy is "+quaternion.getY()+" qz is "+quaternion.getZ()+" qw is "+ quaternion.getW());

        DefineQRCODE(api.getMatNavCam());
        //moveTo1(tabx,taby,tabz,tax,tay,taz);
        //DefineQRCODEBmap(api.getBitmapNavCam());
        runForEachPattern(pattern);
//        imgCirProc = new imgProcessing();
//        imgCirProc.findCircularContours(api.getMatNavCam());
//        Log.i("LOG-INFO","CIRCLE SIZE IS : "+imgCirProc.processedCircleImg.size());
//
        //moveToWrapper(11.21,-9.8,4.79, q.getX(), q.getY(), q.getZ(),  q.getW());
       //moveToWrapper(11.21,-9.8,4.79, -0.240,-0.266, -0.694,  0.625);

        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);
        Log.d("LOG-DEBUGGER","Can move");

        if(pattern == 7){ //X : 11.29 Y : -9.8 Z : 5.
            //moveToWrapper(px,py,pz,  -0.064, -0.083,-0.789  ,0.605);
            moveToWrapper(11.52,py,pz, 0, 0, -0.707, 0.707);
            Log.d("STEP-DEBUGGER","1st step");
            moveToWrapper(11.52, py, 4.79, 0, 0, -0.707, 0.707);
            Log.d("STEP-DEBUGGER","2nd step");
            //moveToWrapper(11.21, -10, 4.85, 0, 0, -0.707, 0.707);

        }
        else if(pattern == 8){
            moveToWrapper(px, py, 4.79, 0, 0, -0.707, 0.707);
        }
        else if(pattern == 1){
            moveToWrapper(11.35, -9.9, 5, 0, 0, -0.707, 0.707);
        }
        else if(pattern == 5 || pattern == 6){
            moveToWrapper(10.5,py,pz, 0, 0, -0.707, 0.707);
        }
        Log.d("STEP-DEBUGGER","START FINAL MOVING");
        moveToWrapper(10.6, -9.0, 4.85,0, 0, -0.707, 0.707);
        Log.d("STEP-DEBUGGER","Final step moving");
        moveToWrapper(10.6, -8.0, 4.5,0, 0, -0.707, 0.707);
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
    {
        Result result;
        int count = 0, max_count = 3;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

        do
        {
            result = api.moveTo(point, quaternion, true);
            count++;
        }
        while (!result.hasSucceeded() && count < max_count);
    }

    public void moveTo1(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des)
    {
        Log.d("LOG-DEBUGGER","POS X"+x_org+" POS Y "+y_org+" POS Z "+z_org+" DES X "+x_des+" DES Y "+y_des+" DES Z "+z_des);
        double dx = x_des-x_org;
        double dy = y_des-y_org;
        double dz = z_des-z_org;
        double magnitude = Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
        double x_unit = dx/magnitude;
        double y_unit = dy/magnitude;
        double z_unit = dz/magnitude;

        double matrix[][] =
                {
                        {1, 0, 0},
                        {x_unit, y_unit, z_unit}
                };

        double x = matrix[0][1]*matrix[1][2] - matrix[1][1]*matrix[0][2];
        double y = matrix[0][2]*matrix[1][0] - matrix[1][2]*matrix[0][0];
        double z = matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1];
        double i = matrix[1][0]-matrix[0][0];
        double j = matrix[1][1]-matrix[0][1];
        double k = matrix[1][2]-matrix[0][2];
        double q = Math.sqrt(x*x + y*y + z*z);
        double p = Math.sqrt(i*i + j*j + k*k);
        double theta = Math.acos((2 - p*p) / 2);

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        double pitch = -Math.atan((2 * (a*w + b*c)) / (w*w - a*a - b*b + c*c));
        double roll = -Math.asin(2 * (a*c - b*w));
        double yaw = Math.atan((2 * (c*w + a*b)) / (w*w + a*a - b*b - c*c));
        double sx = (0.103 * Math.cos(roll + 0.279) / Math.cos(1.57080 + yaw));
        double sy = (0.103 * Math.sin(roll + 0.279) / Math.cos(pitch));
        Log.d("LOG-DEBUGGER","sx : "+sx+ " sy : "+sy);
        Log.d("LOG_DEBUGGER","qx : "+(float)a+" qy : "+(float)b+" qz : "+(float)c+" qw : "+(float)w);
        moveTo((float)x_org - (float)sx, (float)y_org, (float)z_org + (float)sy, (float)a, (float)b, (float)c, (float)w);
        //moveToWrapper(x_org,y_org,z_org,a,b,c,w);
    }


    void ARUCOREADER(Mat arucoMat) {
        corners = new ArrayList<>();
        ids = new Mat();
        dict = Aruco.getPredefinedDictionary(dictID);
        Aruco.detectMarkers(arucoMat, dict, corners, ids);

        borderColor = new Scalar(255.0, 0.0, 0.0);
        double tx = 0;
        double ty = 0;
        if (ids.size().height > 0) {
            Aruco.drawDetectedMarkers(arucoMat, corners, ids, borderColor);

            markerSize = 0.05f;
            rvecs = new Mat();
            tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerSize, camMatrix, dstMatrix, rvecs, tvecs);

            Point3 target_pos = new Point3();
            Point3 marker_pos = new Point3();

            for (int i = 0; i < rvecs.size().height; i++) {
				System.out.println(corners.get(i).dump());
                Mat rvec = new Mat(1, 3, CvType.CV_64FC1);
                Mat tvec = new Mat(1, 3, CvType.CV_64FC1);
                Mat rot = new Mat();
                rvec.put(0, 0, rvecs.get(i, 0));
                tvec.put(0, 0, tvecs.get(i, 0));
                Calib3d.drawFrameAxes(arucoMat, camMatrix, dstMatrix, rvec, tvec, (float) 0.15);

                double _x = 0;
                double _y = 0;
                for (int j = 0; j < corners.size(); j++) {
                    _x = _x + corners.get(i).get(0, j)[0];
                    _y = _y + corners.get(i).get(0, j)[1];
                }
                _x = _x / 4.0;
                _y = _y / 4.0;
                float pos_x = (float) tvec.get(0, 0)[0];
                float pos_y = (float) tvec.get(0, 1)[0];
                float pos_z = (float) tvec.get(0, 2)[0];

                //for debug
//				rvec.put(0, 0, rvecs.get(1,0));
//				tvec.put(0, 0, tvecs.get(1,0));
//
//				marker_pos.x=tvec.get(0, 0)[0];
//				marker_pos.y=tvec.get(0, 1)[0];
//				marker_pos.z=tvec.get(0, 2)[0];
                ///debug

                tx = tx + _x;
                ty = ty + _y;

                target_pos.x = target_pos.x + pos_x;
                target_pos.y = target_pos.y + pos_y;
                target_pos.z = target_pos.z + pos_z;


                String AR_position = String.format("(%.2f,%.2f,%.2f)m", pos_x, pos_y, pos_z);
//				convert rvec to rotation matrix
				Calib3d.Rodrigues(rvec, rot);

                orientation = new quaternion();

                orientation.fromRotationMatrix(rot);
                orientation.toEuler();

				Log.d("LOG-DEBUGGER","AR : "+AR_position);
                Log.d("LOG-DEBUGGER","AR : "+ids.get(i,0)[0]);
                Log.d("LOG-DEBUGGER","roll : "+orientation.roll+" pitch : "+orientation.pitch+" yaw : "+orientation.yaw);

            }

            target_pos.x = target_pos.x / 4.0;
            target_pos.y = target_pos.y / 4.0;
            target_pos.z = target_pos.z / 4.0;
            Point robot = api.getTrustedRobotKinematics().getPosition();
            tax =  robot.getX()+target_pos.x;
            tay = robot.getY()-target_pos.y;
            taz = target_pos.z + robot.getZ();
            String target_position = String.format("(%.2f,%.2f,%.2f)m", target_pos.x, target_pos.y, target_pos.z);
            String cuurent_target_position = String.format("(%.2f,%.2f,%.2f)m", tax, tay,taz);

            Log.d("LOG-DEBUGGER","TARGET"+target_position);
            Log.d("LOG-DEBUGGER","TARGET 1 : "+cuurent_target_position);
            Log.d("LOG-DEBUGGER","TARGET BEE :"+robot.toString());

            MatOfPoint3f target3D = new MatOfPoint3f();
            target3D.fromArray(target_pos);
            MatOfPoint2f targetImagePlane = new MatOfPoint2f();
            Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
            Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);
            double[] r = new double[]{0.0f, 0.0f, 0.0f};
            double[] t = new double[]{0.0f, 0.0f, 0.0f};
            _rvec.put(0, 0, r);
            _tvec.put(0, 0, t);
            double vectA[]={robot.getX(), robot.getY(), robot.getZ()};
            double vectB[]={px, py, pz};
            tabx = robot.getX();
            taby = robot.getY();
            tabz = robot.getZ();
            //q = quaternion.vectorToQuaternion(vectA, vectB);
            //Log.i("LOG-INFO","qw :"+q.getW()+" qx : "+q.getX()+" qy : "+q.getY()+" qz : "+q.getZ());
            Calib3d.projectPoints(target3D, _rvec, _tvec, camMatrix, new MatOfDouble(dstMatrix), targetImagePlane);

                //center of target from 3D-world coordinate
            org.opencv.core.Point _center= new org.opencv.core.Point(targetImagePlane.get(0, 0)[0],targetImagePlane.get(0, 0)[1]);
            Log.i("LOG-INFO","POINT_WORLD "+_center.toString());



            // center of target from image position
            org.opencv.core.Point center = new org.opencv.core.Point(tx, ty);
            Log.i("LOG-INFO","POINT "+center.toString());
        }
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private boolean moveToWrapperNEXTGEN(double pos_x, double pos_y, double pos_z,
                                         double rot_x, double rot_y, double rot_z) {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = eulerToQuaternionAgent(rot_x,rot_y,rot_z);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX)
        {
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }

        return result.hasSucceeded();
    }

    public Quaternion eulerToQuaternionAgent(double pitch_degree,double roll_degree,double yaw_degree)
    {
        double yaw = Math.toRadians(yaw_degree);
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp *cy - cr * sp * sy;
        double qy = cr * sp *cy + sr * cp * sy;
        double qz = cr * cp *sy - sr * sp * cy;
        double qw = cr * cp *cy + sr * sp * sy;

        return new Quaternion((float)qx,(float)qy,(float)qz,(float)qw);
    }


    boolean DefineQRCODE(Mat mat) {
        int thresh = 250;
        String content = readQRImage2(mat,thresh);
        api.sendDiscoveredQR(content);
        String[] thing = content.split("\"");
        //String[] thing = content.split(" ");

        if (content != " ") {
            char p = thing[2].charAt(1);
            Log.d("P IS ",Character.toString(p));
            pattern = p - 48;

            Log.d("LOG-DEBUGGER", "PATTERN : " + pattern);

            px = StringToFloat(thing[4]);
            py = StringToFloat(thing[6]);
            pz = StringToFloat(thing[8]);

            Log.d("LOG-DEBUGGER", "X : " + px + " Y : " + py + "Z : " + pz);

            return true;
        } else {
            return false;
        }
    }

    boolean DefineQRCODEBmap(Bitmap bitmap) {
        String content = readQRImage2Bitmmap(bitmap);
        api.sendDiscoveredQR(content);

        String[] thing = content.split("\"");
        //String[] thing = content.split(" ");

        if (content != " ") {
            char p = thing[2].charAt(1);
            Log.d("P IS ",Character.toString(p));
            pattern = p - 48;

            Log.d("LOG-DEBUGGER", "PATTERN : " + pattern);

            px = StringToFloat(thing[4]);
            py = StringToFloat(thing[6]);
            pz = StringToFloat(thing[8]);

            Log.d("LOG-DEBUGGER", "X : " + px + " Y : " + py + "Z : " + pz);

            return true;
        } else {
            return false;
        }
    }

    public Bitmap resizeImage(Mat src, int width, int height)
    {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }

    //THIS WORK 1.5
    public String readQRImage2(Mat mat, int thresh) {
        String contents = " ";

        int key = 0;
        while (contents == " "){
            Map<DecodeHintType,Object> hint = new HashMap<>();
            hint.put(DecodeHintType.PURE_BARCODE, true);
            Log.d("LOG-DEBUGGER","DEFINED HINT : "+hint.toString());
            imgProc = new imgProcessing();
            imgProc.findRectContours(mat, thresh);
            Mat src_mat;
            if(key >= 1){
                src_mat = api.getMatNavCam();
            }
            else if(!imgProc.STATUS){
                src_mat = mat;
            }
            else {
                src_mat = imgProc.sharpenImg;
            }
            Log.d("LOG-DEBUGGER","MAT COLS IS "+src_mat.cols()+" MAT ROWS IS "+src_mat.rows());
            Bitmap bMap = Bitmap.createBitmap(src_mat.cols(), src_mat.rows(), Bitmap.Config.ARGB_8888);

            matToBitmap(src_mat,bMap);
            Log.d("LOG-DEBUGGER","CONVERT MAT TO BITMAP");
            Log.d("LOG-DEBUGGER","get new bitmap!!!");
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            Reader reader = new QRCodeReader();
            try {
                Log.d("LOG-DEBUGGER", "START DECODE");
                //contents = reader.decode(bitmap, hint).getText();
                contents = reader.decode(bitmap).getText();
                Log.d("LOG-DEBUGGER", "FINISH DECODE");
            } catch (NotFoundException e) {
                e.printStackTrace();
                key++;
            } catch (ChecksumException e) {
                e.printStackTrace();
                key++;
            } catch (FormatException e) {
                e.printStackTrace();
                key++;
            }
        }


        Log.d("LOG-DEBUGGER", "END OF SCAN");
        //ARUCOREADER(imgProc.processedImg);
        return contents;
    }

    public String readQRImage2Bitmmap(Bitmap btmap) {
        String contents = " ";
        Log.d("LOG-DEBUGGER", "START SCAN");
        imgProc = new imgProcessing();

        Bitmap bMap = imgProc.GreyScale(btmap);

        int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

        Reader reader = new QRCodeReader();
        try {
            Log.d("LOG-DEBUGGER", "START DECODE");
            contents = reader.decode(bitmap).getText();
            Log.d("LOG-DEBUGGER", "FINISH DECODE");
        } catch (NotFoundException e) {
            e.printStackTrace();
        } catch (ChecksumException e) {
            e.printStackTrace();
        } catch (FormatException e) {
            e.printStackTrace();
        }

        Log.d("LOG-DEBUGGER", "END OF SCAN");
        return contents;
    }

    //QR INIT
    public String readQRImageDECOY() {
        Log.d("LOG-DEBUGGER", "START DECOY SCAN");
        Bitmap bMap = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
        String contents = " ";

        int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

        Reader reader = new QRCodeReader();
        try {
            Log.d("LOG-DEBUGGER", "START DECODE");
            contents = reader.decode(bitmap).getText();
            Log.d("LOG-DEBUGGER", "FINISH DECODE");
        } catch (NotFoundException e) {
            e.printStackTrace();
        } catch (ChecksumException e) {
            e.printStackTrace();
        } catch (FormatException e) {
            e.printStackTrace();
        }

        Log.d("LOG-DEBUGGER", "END OF SCAN");
        return contents;
    }

    public Float StringToFloat(String s) {
        return Float.parseFloat(s.substring(1, s.length() - 1));
    }

    public void runForEachPattern(int pattern){
        Log.d("LOG-DEBUGGER","Starting");
        if(pattern == 1) {
            //moveToWrapper(px,py,pz, -0.092,-0.114,-0.766,  0.626);
            moveToWrapperNEXTGEN(px, py, pz, -16.478,3.555,-102.001);
        }
        else if(pattern == 2){
            //moveToWrapper(px,py,pz, -0.102,-0.098,-0.689,0.711);
            moveToWrapperNEXTGEN(px, py, pz, -16.251,-0.597,-88.114);
        }
        else if(pattern == 3){
//            moveToWrapper(px, -9.8, 5, -0.100,-0.094,-0.677,0.723);
//            moveToWrapper(px,py,pz, -0.100,-0.094,-0.677,0.723);
            moveToWrapperNEXTGEN(px, -9.8, 5, -15.745,-1.031,-86.094);
            moveToWrapperNEXTGEN(px, py, pz, -15.745,-1.031,-86.094);
        }
        else if(pattern == 4){

            //moveToWrapper(px, -9.8, 5, -0.105,-0.099,-0.679,0.720);
            //moveToWrapper(px,py,pz, -0.105,-0.099,-0.679,0.720);

            moveToWrapperNEXTGEN(px, -9.8, 5.0,-16.563,-1.002,-86.497);
            moveToWrapperNEXTGEN(px, py, pz, -16.25,-1.002,-86.497);

        }
        else if(pattern == 5){
//            moveToWrapper(10.5,py,4.79, 0,0,-1,0);
//            moveToWrapper(10.5,py,pz, 0, 0, -0.707, 0.707);
//            moveToWrapper(px,py,pz,  -0.006, -0.006,-0.688,  0.725);

            moveToWrapperNEXTGEN(px - .25, -9.8, 4.9,-0.1,-0.972,-85.75);
            moveToWrapperNEXTGEN(px - .25, -9.8, pz,-0.1,-0.972,-85.75);
            moveToWrapperNEXTGEN(px, py, pz,-0.1,-0.972,-85.75);
        }
        else if(pattern == 6){
//            moveToWrapper(10.5,py,4.79, -0.004,  -0.004,-0.672,  0.740);
//            moveToWrapper(10.5,py,pz, -0.004,  -0.004,-0.672,  0.740);
//            moveToWrapper(px,py,pz,  -0.004,  -0.004,-0.672,  0.740);

            moveToWrapperNEXTGEN(px - .25, -9.8, 4.9,  -0.031, -0.648, -84.486);
            moveToWrapperNEXTGEN(px - .25, -9.8, pz,  -0.031, -0.648, -84.486);
            moveToWrapperNEXTGEN(px, py, pz,  -0.648, -0.031, -84.486);
        }
        else if(pattern == 7){

//            moveToWrapper(11.52, py, 4.79, 0, 0, -0.707, 0.707);
//            moveToWrapper(11.52,py,pz, 0, 0, -0.707, 0.707);
//            moveToWrapper(px,py,pz,    -0.006,   -0.008,-0.777 ,  0.629);

            moveToWrapperNEXTGEN(11.52, -9.8, 5,0,0,-90);
            moveToWrapperNEXTGEN(11.52, -9.8, pz,0,0,-90);
            moveToWrapperNEXTGEN(px, py, pz,  -0.67,   0.280, -102.021);
        }
        else if(pattern == 8){
            //moveToWrapper(px,py,pz, -0.093,-0.112,-0.763,0.629);
            //moveToWrapperNEXTGEN(px, py, pz, -15,0,-105);
            //moveToWrapperNEXTGEN(px, py, pz, -15,0,-105);
            moveToWrapperNEXTGEN(px, py, pz,-16.445, 3.226,-101.463);
        }
        Log.d("LOG-DEBUGGER","Moving Complete");
    }
}

