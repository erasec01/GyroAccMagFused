
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

            //following code is taken and adapted from android documentation:
            //https://developer.android.com/reference/android/hardware/SensorEvent

            if (timestamp != 0) {

                //calculate time lapsed from consecutive sampling and convert from nanosecond to second
                final float dT = (event.timestamp - timestamp) * NANOTOSEC;

                //To create quaternion we need:
                //  -Magnitude of rotation vector normalized
                //  -Angle of Rotation vector

                //Calculate the resultant magnitude vector in 3d space
                float resultantGyroVector = (float) Math.sqrt(mGyroscopeData[0] * mGyroscopeData[0] +
                        mGyroscopeData[1] * mGyroscopeData[1] +
                        mGyroscopeData[2] * mGyroscopeData[2]);

                //Normalization of rotation vector
                if (resultantGyroVector > EPSILON) {

                    normGyroVector[0] = mGyroscopeData[0] / resultantGyroVector;
                    normGyroVector[1] = mGyroscopeData[1] / resultantGyroVector;
                    normGyroVector[2] = mGyroscopeData[2] / resultantGyroVector;

                }

                //Calculate rotation angle

                float angleComputed = resultantGyroVector * dT / 2.0f;
                float sinAngleComputed = (float) Math.sin(angleComputed);
                float cosAngleComputed = (float) Math.cos(angleComputed);

                /*
                Create quaternion. By definition a quaternion is composed of 4 elements:
                  -Rotation along X axis
                  -Rotation along Y axis
                  -Rotation along Z axis
                  -Magnitude of rotation
                */
                deltaRotationVector[0] = sinAngleComputed * normGyroVector[0];
                deltaRotationVector[1] = sinAngleComputed * normGyroVector[1];
                deltaRotationVector[2] = sinAngleComputed * normGyroVector[2];
                deltaRotationVector[3] = cosAngleComputed;


                //From quaternion above calculate the corresponding rotation matrix
                SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);

                /*
                /At this point the gyro data are referenced to sensor body system  that is different
                 from inertial reference system in which magAccPosition data is computed.
                 We need to align gyro data with magAccData. To do so we multiply gyroRotationMatrix with
                 magAccRotationMatrix that is already aligned with inertial reference system.
                 In this way gyroRotationMatrix it will be aligned with inertial reference system as well
                 This step is done only the first time gyro event occurs and only if magAccPosition is available
                */
                    if (initState && magAccPosOk) {

                        gyroRotationMatrix = matrixMultiplication(magAccRotationMatrix, gyroRotationMatrix );
                        initState = false;
                    }



                /*
                Update gyroRotationMatrix with actual gyro value rotation. Here is where integration
                is performed. gyroRotationMatrix is updated with current rotation values and aligned
                with inertial reference system
                */
                gyroRotationMatrix = matrixMultiplication(deltaRotationMatrix, gyroRotationMatrix );

                /*
                Get position from gyro rotation matrix using android's method getOrientation()
                (Here gyro position is affected by drift). The calculated position is used in the
                final fused position
                */
                SensorManager.getOrientation(gyroRotationMatrix, gyroPosition);


                /*
                write in textview only if this class is used as stand alone APP
                */
                mTextgyroX.setText(getResources().getString(
                        R.string.value_format, gyroPosition[0]));

                mTextgyroY.setText(getResources().getString(
                        R.string.value_format, gyroPosition[1]));

                mTextgyroZ.setText(getResources().getString(
                        R.string.value_format, gyroPosition[2]));



            }

            //update timestamp
            timestamp = event.timestamp;


            /*
            Calculate fused position and apply complementary filter
            --> if filter_coeff is near 1 Gyro values wins, if is near 0 magAcc wins
            */
            fusedPosition[0] = filter_coeff*gyroPosition[0] + (1.0f-filter_coeff)*magAccPosition[0];
            fusedPosition[1] = filter_coeff*gyroPosition[1] + (1.0f-filter_coeff)*magAccPosition[1];
            fusedPosition[2] = filter_coeff*gyroPosition[2] + (1.0f-filter_coeff)*magAccPosition[2];

           /* gyroPosition[0] = fusedPosition[0];
            gyroPosition[1] = fusedPosition[1];
            gyroPosition[2] = fusedPosition[2];
            */
            /*
                Until here fused position is affected by drift generated by gyro.
                To eliminate drift, gyro should continue to follow the value provided by magAcc that
                is without drift.
                To do so we update gyroRotationMatrix with new rotation matrix calculated by
                actual fused position. In this way the resultant gyroRotationMatrix
                contains "rotation instructions" in order to move new gyro position affected by drift
                versus magAcc position that is without drift. Method getRotationMatrixFromPosition is used to calculate
                rotation matrix from euler angles (actual position) passed by arguments. The vector order passed is swapped
                since the method is expected to receive X Y Z angle in order, but fusedPosition vector
                has Y Z X order instead
            */
            gyroRotationMatrix = getRotationMatrixFromAngle(fusedPosition[1], fusedPosition[2], fusedPosition[0]);



            /*
                Write in textview only if this class is used as stand alone APP
            */
            mTextFusedX.setText(getResources().getString(
                    R.string.value_format, fusedPosition[0]));

            mTextFusedY.setText(getResources().getString(
                    R.string.value_format, fusedPosition[1]));

            mTextFusedZ.setText(getResources().getString(
                    R.string.value_format, fusedPosition[2]));

        }
    }

    /**
     * This method calculate the Rotation Matrix corresponding to a given position passed by argument
     * The matrix calculated is the combination of the 3 rotation matrix corresponding to rotation
     * of X Y Z vectors. The convention used for calculation is ENU, right hand rule. Angles X Y are
     * considered positive with counterclockwise rotation, angle Z is considered negative with
     * counterclockwise rotation.
     * @param angleX    rotation along X axis in rad
     * @param angleY    rotation along Y axis in rad
     * @param angleZ    rotation along Z axis in rad
     * @return  <code>float[] resultMatrix</code> The rotation matrix corresponding to position passed to the method

    */

    private float[] getRotationMatrixFromAngle(float angleX, float angleY, float angleZ) {

        float[] rm = new float[9];

        float sinX = (float)Math.sin(angleX);
        float cosX = (float)Math.cos(angleX);

        float sinY = (float)Math.sin(angleY);
        float cosY = (float)Math.cos(angleY);

        float sinZ = (float)Math.sin(angleZ);
        float cosZ = (float)Math.cos(angleZ);


        // rotation about x-axis (pitch)
        rm[0] = (cosZ*cosY)-(sinX*sinY*sinZ); rm[1] = cosX*sinZ; rm[2] = (cosZ*sinY)+(sinZ*sinX*cosY) ;
        rm[3] = 0.0f-(sinZ*cosY)-(sinX*sinY*cosZ); rm[4] = cosZ*cosX; rm[5] = (cosZ*sinX*cosY)-(sinZ*sinY);
        rm[6] = 0.0f-(sinY*cosX); rm[7] = 0.0f-sinX; rm[8] = cosX*cosY;

        return rm;
    }




    /**
     * This method performs matrix multiplication, it should be re-emplemented using
     * Strassen algorithm in order to get better performance
     *
     * @param A Matrix to be multiplied with B
     * @param B Matrix to be multiplied with A
     * @return <code>float[]</code> The resulting matrix from A*B multiplication

     */
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

    /**
     * This method calculates 3x3 identity matrix
     *
     * @param m The matrix to be converted into identity matrix
     */

    private void getIdentityMatrix(float[] m ){


        for (int i = 0; i<m.length; i++){

            if(i==0 || i==4 || i==8)
                m[i]=1;
            else
                m[i]=0;

        }


    }





}
