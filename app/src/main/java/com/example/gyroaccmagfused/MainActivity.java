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


    public static final float EPSILON = 0.000000001f;

    private float angleComputed;


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

    //nanosecond
    private static final float NANOTOSEC = 1.0f/1000000000.0f;

    private float timestamp;

    private float resultantGyroVector;

    //matrix & vector:
    private float[] mGyroscopeData = new float[3];
    private float[] mAccelerometerData = new float[3];
    private float[] mMagnetometerData = new float[3];
    private float[] magAccPosition = new float[3];




    private float[] normGyroVector = new float[3];



    private float[] deltaRotationVector = new float[4];
    private float[] deltaRotationMatrix = new float[9];
    private float[] gyroRotationMatrix = new float[9];
    private float[] magAccRotationMatrix = new float[9];
    private float[] gyroRotationReferenced = new float[9];
    private float[] actualRotationReference = new float[9];



    private float[] gyroPosition = new float[3];
    private float[] fusedPosition = new float[3];
    private boolean initState = true;

    private float filter_coeff = 0.98f;


    // test variable
    private int posCountTest = 0;
    private int rotMatTest = 0;
    private boolean magAccPosOk = false;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

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

        boolean rotationOk = SensorManager.getRotationMatrix(magAccRotationMatrix, null, mAccelerometerData, mMagnetometerData);

        if(rotationOk) {
            magAccPosition = SensorManager.getOrientation(magAccRotationMatrix, magAccPosition);
            magAccPosOk = true;

        }

        mTextMagAccX.setText(getResources().getString(
                R.string.value_format, magAccPosition[0]));

        mTextMagAccY.setText(getResources().getString(
                R.string.value_format, magAccPosition[1]));

        mTextMagAccZ.setText(getResources().getString(
                R.string.value_format, magAccPosition[2]));




        //here only if sensor is gyro, this check is needed for timestamp consistency


        if(sensorType == Sensor.TYPE_GYROSCOPE) {

            if (timestamp != 0) {

                final float dT = (event.timestamp - timestamp) * NANOTOSEC;
                float a = 1.0f;


                //following code is taken and adapted from android documentation "Sensor event Gyroscope"


                //calculate time lapsed from consecutive sampling and convert from nanosecond to second


                //calculate the resultant vector in 3d space

                resultantGyroVector = (float) Math.sqrt(mGyroscopeData[0] * mGyroscopeData[0] +
                        mGyroscopeData[1] * mGyroscopeData[1] +
                        mGyroscopeData[2] * mGyroscopeData[2]);

                //normalization of rotation vector

                if (resultantGyroVector > EPSILON) {

                    normGyroVector[0] = mGyroscopeData[0] / resultantGyroVector;
                    normGyroVector[1] = mGyroscopeData[1] / resultantGyroVector;
                    normGyroVector[2] = mGyroscopeData[2] / resultantGyroVector;

                }

                //create quaternion

                angleComputed = resultantGyroVector * dT / 2.0f;
                float sinAngleComputed = (float) Math.sin(angleComputed);
                float cosAngleComputed = (float) Math.cos(angleComputed);

                //four quaternion's elements:
                deltaRotationVector[0] = sinAngleComputed * normGyroVector[0];
                deltaRotationVector[1] = sinAngleComputed * normGyroVector[1];
                deltaRotationVector[2] = sinAngleComputed * normGyroVector[2];
                deltaRotationVector[3] = cosAngleComputed;


                //calculate rotation matrix from quaternion

                SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);

                rotMatTest++;
                // //align gyro with inertial frame reference



                    if (initState && magAccPosOk) {


                            gyroRotationMatrix = matrixMultiplication(gyroRotationMatrix, magAccRotationMatrix);

                            initState = false;
                    }





                //update gyroRotationMatrix with actual rotation. Here integration is performed

                gyroRotationMatrix = matrixMultiplication(gyroRotationMatrix, deltaRotationMatrix);

                // get gyro position from Rotation matrix
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



            fusedPosition[0] = filter_coeff*gyroPosition[0] + (1.0f-filter_coeff)*magAccPosition[0];
            fusedPosition[1] = filter_coeff*gyroPosition[1] + (1.0f-filter_coeff)*magAccPosition[1];
            fusedPosition[2] = filter_coeff*gyroPosition[2] + (1.0f-filter_coeff)*magAccPosition[2];

            gyroRotationMatrix = getRotationMatrixFromOrientation(fusedPosition);

            mTextFusedX.setText(getResources().getString(
                    R.string.value_format, fusedPosition[0]));

            mTextFusedY.setText(getResources().getString(
                    R.string.value_format, fusedPosition[1]));

            mTextFusedZ.setText(getResources().getString(
                    R.string.value_format, fusedPosition[2]));

        }





        // calculate Accelerometer Magnetometer position
        //calculate rotation Matrix from accelerometer and magnetometer
        /*
        ..............
         */

        //Calculate new Quaternions from Gyrodata
        /*
        .............
         */

        //Converts Quaternions to rotation matrix using getRotationMatrixFromVector()
        /*
        ....
         */
        //Calculate new gyro Rotation Matrix multiplying MagACC rotation matrix with Rotation matrix from quaternion above
        /*
        ....
         */

        //Calculate new orientation value with method getOrientation()
        /*
        ....
         */
        



    }


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

    private void getIdentityMatrix(float[] m ){


        for (int i = 0; i<m.length; i++){

            if(i==0 || i==4 || i==8)
                m[i]=1;
            else
                m[i]=0;

        }


    }





}
