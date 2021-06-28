package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.CvType;
import org.opencv.core.Mat;



public class camera_params {
    private Mat cameraMatrix;
    private Mat dstMatrix;

    public Mat getCamMatrix() {
        return cameraMatrix;
    }

    public Mat getDistortionMat() {
        return dstMatrix;
    }


    public void process_camera() {
        double fx=567.22931;
        double cx=659.07721;
        double fy=574.19293;
        double cy=517.00757;

        double[] camMatrix = new double[] {fx,0,cx,0.0,fy,cy,0.0,0.0,1.0};
        double[] distMatrix = new double[] {-0.21624701, 0.03875, -0.010157, 0.0019690001, 0};


        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, camMatrix);// (row, col,int[])

        dstMatrix=new Mat(1,5,CvType.CV_64FC1);
        dstMatrix.put(0, 0,distMatrix);


    }
}
