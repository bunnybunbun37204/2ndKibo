package jp.jaxa.iss.kibo.rpc.sampleapk;


import org.opencv.core.Mat;

import gov.nasa.arc.astrobee.types.Quaternion;

public class quaternion {
    public static float x=0;
    public static float y=0;
    public static float z=0;
    public static float w=1;

    public static float roll=0;
    public static float pitch=0;
    public static float yaw=0;
    private static float M_PI=3.1416f;

    public static void crossProduct(double vect_A[], double vect_B[], double cross_P[])

    {

        cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
        cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
        cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    }

    public static double dotProduct(double vect_A[], double vect_B[])
    {

        double product = 0;

        // Loop for calculate dot product
        for (int i = 0; i < 3; i++)
        {
            product = product + vect_A[i] * vect_B[i];
        }
        return product;
    }

    public static double lengthVector(double vector[])
    {
        double a = 0;
        for (int i = 0; i < 3; i++)
        {
            a += Math.pow(vector[i], 2);
        }
        double result = Math.sqrt(a);
        return result;
    }

    public static double lengthVectorQuaternion(double vector[])
    {
        double a = 0;
        for (int i = 0; i < 4; i++)
        {
            a += Math.pow(vector[i], 2);
        }
        double result = Math.sqrt(a);
        return result;
    }

    public static void normalization(double vector[], double outputVector[])
    {
        double length = lengthVector(vector);
        double []unit_vector = new double[3];
        for (int i = 0; i < 3; i++)
        {
            unit_vector[i] = (1 / length) * vector[i];
            outputVector[i] = unit_vector[i];
        }
    }

    public static Quaternion normalizationQuaternion(Quaternion q)
    {
        double qua[] = {q.getW(), q.getW(), q.getY(), q.getZ()};
        float d = (float)lengthVectorQuaternion(qua);
        float qw = q.getW()/d;
        float qx = q.getX()/d;
        float qy = q.getY()/d;
        float qz = q.getZ()/d;
        Quaternion norm_qua = new Quaternion(qx, qy, qz, qw);
        return norm_qua;
    }

    public static Quaternion vectorToQuaternion(double vectorA[], double vectorB[]){
        Quaternion q;
        double []cross_p = new double[3];
        double []normA = new double[3];
        double []normB = new double[3];
        crossProduct(vectorA, vectorB, cross_p);
        normalization(vectorA, normA);
        normalization(vectorB, normB);
        float x = (float) cross_p[0];
        float y = (float) cross_p[1];
        float z = (float) cross_p[2];
        double m1 = lengthVector(normA);
        double m2 = lengthVector(normB);
        double w = Math.sqrt(m1*m1*m2*m2)+dotProduct(normA, normB);
        double magni = Math.sqrt(x*x+y*y+z*z);
        double qw = w/magni;
        double qx = x/magni;
        double qy = y/magni;
        double qz = z/magni;
        q = new Quaternion((float)qx, (float)qy*-1, (float)qz*-1, (float)qw);
        return q;
    }

    public static void toEuler() {
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = (float) Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = 2 * (w * y - z * x);
        if (Math.abs(sinp) >= 1)
            pitch = Math.copySign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = (float) Math.asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = (float) Math.atan2(siny_cosp, cosy_cosp);
    }

    public static void fromRotationMatrix(Mat rot) {
        int row=3;
        int col=3;
        float[][] m=new float[3][3];

        for(int i=0;i<row;i++) {
            for(int j=0;j<col;j++) {
                m[i][j]=(float) rot.get(i,j)[0];
            }
        }
        float tr=m[0][0]+m[1][1]+m[2][2];

        if(tr>0.0) {
            float S = (float)Math.sqrt((float)tr+1.0) * 2;
            w=(float)0.25*S;
            x = (m[2][1] - m[1][2]) / S;
            y = (m[0][2] - m[2][0]) / S;
            z = (m[1][0] - m[0][1]) / S;
        }
        else if((m[0][0]>m[1][1])&(m[0][0]>m[2][2])) {
            float S = (float)Math.sqrt(1.0+m[0][0]-m[1][1]-m[2][2]) * 2;
            w=(m[2][1] - m[1][2]) / S;
            x=(float)0.25*S;
            y = (m[0][1] + m[1][0]) / S;
            z = (m[0][2] + m[2][0]) / S;
        }
        else if (m[1][1] > m[2][2]) {
            float S = (float)Math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
            w = (m[0][2] - m[2][0]) / S;
            x = (m[0][1] + m[1][0]) / S;
            y = (float)0.25 * S;
            z = (m[1][2] + m[2][1]) / S;
        }
        else {
            float S = (float)Math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
            w = (m[1][0] - m[0][1]) / S;
            x = (m[0][2] + m[2][0]) / S;
            y = (m[1][2] + m[2][1]) / S;
            z = (float)0.25 * S;
        }
    }
}