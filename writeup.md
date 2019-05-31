## Project: Building an Estimator
### Marko Sarkanj
![Quad Image](./misc/screenshot_1.png)

---


### Writeup / README

This writeup contains the description of implementation of the Building an Estimator project. This project has been completed as part of the Self Flying Car Nanodegree program from Udacity.

### Code Implementation

#### 1. Determining the standard deviation of the measurement noise of both GPS X data and Accelerometer X data

The standard deviation of the measurement noise of both GPS X data and Accelerometer X data has been calculated with the help of `Graph1.txt` and `Graph2.txt` log files and the `STDEV` Excel formula. 

![STDEV Example](./writeup_images/STDEV.png)

The standard deviation determined in the described way is capturing ~68% of the sensor measurements. 

#### 2. Implementing a better rate gyro attitude integration scheme in the UpdateFromIMU() function

The `UpdateFromIMU()` function has been implemented with the help of quaternions. First `bodyRateEst` quaternion has been created with the help of `FromEuler123_RPY()` function. Input of that function are roll, pitch and yaw estimates. Afterwards the `IntegrateBodyRate()` function has been called to integrate the gyroscope measurements into the `bodyRateEst` quaternion. Then the new roll, pitch and yaw values have been extracted by calling the appropriate functions on the `bodyRateEst` quaternion. As the last step the yaw value has been normalized to value that represents rotations up to the radius of one circle.

```cpp
  Quaternion<float> bodyRateEst;
  V3D bodyRateGyro{static_cast<double>(gyro.x), static_cast<double>(gyro.y), static_cast<double>(gyro.z)};

  bodyRateEst = bodyRateEst.FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
  bodyRateEst = bodyRateEst.IntegrateBodyRate(bodyRateGyro, static_cast<double>(dtIMU));

  float predictedPitch = bodyRateEst.Pitch();
  float predictedRoll = bodyRateEst.Roll();
  ekfState(6) = bodyRateEst.Yaw();

  // normalize yaw
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f * F_PI;
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f * F_PI;
```

#### 3. Implementing all of the elements of the prediction step for the estimator

The update step `PredictState()` has been implemented according to the `7.2 Transition Model` from the Estimation for Qadrotors paper provided by Udacity. The first step is to convert the attitude from the local to global frame. This has been achieved with the `Rotate_BtoI()` function called on the `Quaternion` object. 

The next step is to update the `predictedState` vector for the time `dt` multiplied by the velocity contained in the `currState` vector. The velocity has been updated in the  `predictedState` vector by multiplying 

```cpp
    V3F accelInertFrame = attitude.Rotate_BtoI(accel);
    predictedState[0] += curState[3] * dt;
    predictedState[1] += curState[4] * dt;
    predictedState[2] += curState[5] * dt;

    predictedState[3] += accelInertFrame.x * dt;
    predictedState[4] += accelInertFrame.y * dt;
    predictedState[5] += (accelInertFrame.z - static_cast<float>(CONST_GRAVITY)) * dt;
```
```cpp
    float sinPitch = sin(pitch);
    float cosPitch = cos(pitch);

    float sinRoll = sin(roll);
    float cosRoll = cos(roll);

    float sinYaw = sin(yaw);
    float cosYaw = cos(yaw);

    RbgPrime(0,0) = - cosPitch * sinYaw;
    RbgPrime(0,1) = - sinRoll * sinPitch * sinYaw - cosRoll * cosYaw;
    RbgPrime(0,2) = - cosRoll * sinPitch * sinYaw + sinRoll * cosYaw;
    RbgPrime(1,0) = cosPitch * cosYaw;
    RbgPrime(1,1) = sinRoll * sinPitch * cosYaw - cosRoll * sinYaw;
    RbgPrime(1,2) = cosRoll * sinPitch * cosYaw + sinRoll * sinYaw;
```

```cpp
    gPrime(0,3) = dt;
    gPrime(1,4) = dt;
    gPrime(2,5) = dt;
    gPrime(3,6) = (RbgPrime(0) * accel).sum() * dt;
    gPrime(4,6) = (RbgPrime(1) * accel).sum() * dt;
    gPrime(5,6) = (RbgPrime(2) * accel).sum() * dt;

    ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

#### 4. Implementing the magnetometer update

#### 5. Implementing the GPS update


#### 6. De-tuning controller to successfully fly the final desired box trajectory with your estimator and realistic sensors
