
/*
 This class calculate device position from accelerometer, magnetometer and gyroscope using
 sensor fusion and complementary filter approach
 The algorithm steps are as follows:

    Calculate device's position using magnetometer and accelerometer data using android's
    SensorManager method getOrientation()


    Calculate Quaternion from gyro data as reported on: https://developer.android.com/reference/android/hardware/SensorEvent

    Converts quaternion to rotation matrix using android's method getRotationMatrixFromVector()

    Align gyro data to inertial reference system (same as magAcc data), to do so generate magAcc
    rotation matrix and multiply with gyroRotationMatrix. This is done only first time gyro position is computed.

    Calculate rotation matrix corresponding to rotation calculated by gyroscope data. This done by
    multiplying gyroRotationMatrix with rotation matrix calculated from quaternion

    Calculate gyro position using android's method getOrientation()

    Calculate fused position adding magAcc position data with gyro position data and apply complementary filter

    Calculate rotation matrix corresponding to actual fused position and multiply with gyroRotationMatrix
    in order to eliminate Drift from gyro position.

*/




package com.example.gyroaccmagfused;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.TextView;

import static android.util.Half.EPSILON;

public class MainActivity extends AppCompatActivity implements SensorEventListener {


    /**
     * This constant is used to decide when Rotation vector is big enough to start computing quaternion
     */
    public static final float EPSILON = 0.000000001f;


    /**
     * define Text view used as data control. Use these only when class is in main activity
     */
    private TextView mTextgyroX;
    private TextView mTextgyroY;
    private TextView mTextgyroZ;
    private TextView mTextMagAccX;
    private TextView mTextMagAccY;
    private TextView mTextMagAccZ;
    private TextView mTextFusedX;
    private TextView mTextFusedY;
    private TextView mTextFusedZ;


    private SensorManager mSensorManager;
    private Sensor mSensorGyro;
    private Sensor mSensorAcc;
    private Sensor mSensorMag;

    /**
     * Constant to perform conversion from nanosecond to second
     */

    private static final float NANOTOSEC = 1.0f/1000000000.0f;

    /**
     * This timestamp is used to calculate integration time
     */

    private float timestamp;



    /**
     * Matrix and Vectors definition
     * mGyroscopeData, mAccelerometerData, mMagnetometerData = raw data from sensors
     * magAccPosition = position computed from accelerometer and magnetometer
     * gyroPosition = position computed from gyroscope
     * normGyroVector = Orientation vector normalized
     * deltaRotationVector = quaternion vector
     * deltaRotationMatrix = Rotation matrix calculated from quaternion
     * magAccRotationMatrix = Rotation matrix for magnetometer and accelerometer data
     * gyroRotationMatrix = Actual Rotation matrix for gyro contains drift corrections
     **/

    private float[] mGyroscopeData = new float[3];
    private float[] mAccelerometerData = new float[3];
    private float[] mMagnetometerData = new float[3];
    private float[] magAccPosition = new float[3];
    private float[] gyroPosition = new float[3];
    private float[] fusedPosition = new float[3];
    private float[] normGyroVector = new float[3];
    private float[] deltaRotationVector = new float[4];
    private float[] deltaRotationMatrix = new float[9];
    private float[] gyroRotationMatrix = new float[9];
    private float[] magAccRotationMatrix = new float[9];

    /**
     * initState is used to access if statement only during first gyro sensor event
     * magAccPosOk is used to access if statement only if there is a valid position obtained from acc and mag
     */
    private boolean initState = true;
    private boolean magAccPosOk = false;


    /**
     * filter coeff is used in complementary filter to weight gyro data or magAcc data on final result
     */
    private float filter_coeff = 0.98f;






    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //lock the screen in PORTRAIT mode
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);


        // Initialize gyroRotationMatrix to identity matrix in order to permits matrix multiplications
        getIdentityMatrix(gyroRotationMatrix);




        mTextgyroX = (TextView)findViewById(R.id.gyroX);
        mTextgyroY = (TextView)findViewById(R.id.gyroY);
        mTextgyroZ = (TextView)findViewById(R.id.gyroZ);
        mTextMagAccX = (TextView)findViewById(R.id.magAccX);
        mTextMagAccY = (TextView)findViewById(R.id.magAccY);
        mTextMagAccZ = (TextView)findViewById(R.id.magAccZ);
        mTextFusedX = (TextView)findViewById(R.id.fusedX);
        mTextFusedY = (TextView)findViewById(R.id.fusedY);
        mTextFusedZ = (TextView)findViewById(R.id.fusedZ);


        //Initialize SensorManager and Sensor variable
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mSensorMag = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);






    }




    @Override
    protected void onStart(){
        super.onStart();

        if(mSensorGyro != null) {
            mSensorManager.registerListener(this, mSensorGyro, SensorManager.SENSOR_DELAY_FASTEST);
        }

        if(mSensorAcc != null) {
            mSensorManager.registerListener(this, mSensorAcc, SensorManager.SENSOR_DELAY_FASTEST);
        }

        if(mSensorMag != null) {
            mSensorManager.registerListener(this, mSensorMag, SensorManager.SENSOR_DELAY_FASTEST);
        }


    }

    @Override
    protected void onStop(){
        super.onStop();

        mSensorManager.unregisterListener(this);

    }



    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy){

    }

    @Override
    public void onSensorChanged(SensorEvent event){

        //define sensor type variable and read from sensor copying data in the right variable using
        // switch-case construct

        int sensorType = event.sensor.getType();

        switch (sensorType) {
            case Sensor.TYPE_GYROSCOPE:
                mGyroscopeData = event.values.clone();
            case Sensor.TYPE_ACCELEROMETER:
                mAccelerometerData = event.values.clone();
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                mMagnetometerData = event.values.clone();
                break;
            default:
                return;
        }

        // calculate magAccRotationMatrix and magAccPosition, set magAccPosOk == true
        // this position is magnetometer-Accelerometer part of final fused position

        boolean rotationOk = SensorManager.getRotationMatrix(magAccRotationMatrix, null, mAccelerometerData, mMagnetometerData);

        if(rotationOk) {
            magAccPosition = SensorManager.getOrientation(magAccRotationMatrix, magAccPosition);
            magAccPosOk = true;

        }

        //write in textview only if this class is used as stand alone APP

        mTextMagAccX.setText(getResources().getString(
                R.string.value_format, magAccPosition[0]));

        mTextMagAccY.setText(getResources().getString(
                R.string.value_format, magAccPosition[1]));

        mTextMagAccZ.setText(getResources().getString(
                R.string.value_format, magAccPosition[2]));




        // The following code is accessed only if sensor is gyro,
        // Basically this check is needed for timestamp consistency in order to calculate
        // dT only between gyro events


        if(sensorType == Sensor.TYPE_GYROSCOPE) {

            //following code is taken and adapted from android documentation "Sensor event Gyroscope"
            if (timestamp != 0) {

                final float dT = (event.timestamp - timestamp) * NANOTOSEC;






                //calculate time lapsed from consecutive sampling and convert from nanosecond to second


                //calculate the resultant vector in 3d space

                float resultantGyroVector = (float) Math.sqrt(mGyroscopeData[0] * mGyroscopeData[0] +
                        mGyroscopeData[1] * mGyroscopeData[1] +
                        mGyroscopeData[2] * mGyroscopeData[2]);

                //normalization of rotation vector

                if (resultantGyroVector > EPSILON) {

                    normGyroVector[0] = mGyroscopeData[0] / resultantGyroVector;
                    normGyroVector[1] = mGyroscopeData[1] / resultantGyroVector;
                    normGyroVector[2] = mGyroscopeData[2] / resultantGyroVector;

                }

                //create quaternion

                float angleComputed = resultantGyroVector * dT / 2.0f;
                float sinAngleComputed = (float) Math.sin(angleComputed);
                float cosAngleComputed = (float) Math.cos(angleComputed);

                //four quaternion's elements:
                deltaRotationVector[0] = sinAngleComputed * normGyroVector[0];
                deltaRotationVector[1] = sinAngleComputed * normGyroVector[1];
                deltaRotationVector[2] = sinAngleComputed * normGyroVector[2];
                deltaRotationVector[3] = cosAngleComputed;


                //calculate rotation matrix from quaternion

                SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);


                // align gyro with inertial frame reference



                    if (initState && magAccPosOk) {


                            gyroRotationMatrix = matrixMultiplication(gyroRotationMatrix, magAccRotationMatrix);

                            initState = false;
                    }





                //update gyroRotationMatrix with actual gyro value rotation. Here integration is performed

                gyroRotationMatrix = matrixMultiplication(gyroRotationMatrix, deltaRotationMatrix);

                // get gyro position from Rotation matrix. Here Gyro position is affected by Drift
                SensorManager.getOrientation(gyroRotationMatrix, gyroPosition);



                mTextgyroX.setText(getResources().getString(
                        R.string.value_format, gyroPosition[0]));

                mTextgyroY.setText(getResources().getString(
                        R.string.value_format, gyroPosition[1]));

                mTextgyroZ.setText(getResources().getString(
                        R.string.value_format, gyroPosition[2]));



            }

            //update timestamp
            timestamp = event.timestamp;


            //calculate fused position and apply complementary filter --> if filter coeff = 1 Gyro values wins,  elsewhere MagAcc wins
            fusedPosition[0] = filter_coeff*gyroPosition[0] + (1.0f-filter_coeff)*magAccPosition[0];
            fusedPosition[1] = filter_coeff*gyroPosition[1] + (1.0f-filter_coeff)*magAccPosition[1];
            fusedPosition[2] = filter_coeff*gyroPosition[2] + (1.0f-filter_coeff)*magAccPosition[2];


            // Until here fused position is affected by drift generated by Gyro. To eliminate drift we shall continue to follow value provided by magAcc
            //to do so we update gyroRotationMatrix with new rotation matrix calculated by actual fused position. In this way the resultant gyroRotationMatrix
            // contains "rotation instructions" in order to move new Gyro position affected by drift versus magAcc position that is without drift

            gyroRotationMatrix = getRotationMatrixFromOrientation(fusedPosition);

            mTextFusedX.setText(getResources().getString(
                    R.string.value_format, fusedPosition[0]));

            mTextFusedY.setText(getResources().getString(
                    R.string.value_format, fusedPosition[1]));

            mTextFusedZ.setText(getResources().getString(
                    R.string.value_format, fusedPosition[2]));

        }






        



    }

    //this method calculate the Rotation Matrix corresponding to a  given position passed by argument

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    //this method performs matrix multiplication, it should be re-emplemented using Strassen algorithm in order to get better performance
    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    //This method calculates 3x3 identity matrix

    private void getIdentityMatrix(float[] m ){


        for (int i = 0; i<m.length; i++){

            if(i==0 || i==4 || i==8)
                m[i]=1;
            else
                m[i]=0;

        }


    }





}
