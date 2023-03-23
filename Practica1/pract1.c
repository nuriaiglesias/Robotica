#pragma config(Sensor, S1, touchSensor, sensorEV3_Touch)
#pragma config(Sensor, S2, gyroSensor, sensorEV3_Gyro, modeEV3Gyro_RateAndAngle)
#pragma config(Sensor, S3, colorSensor, sensorEV3_Color, modeEV3Color_Ambient)
#pragma config(Sensor, S4, sonarSensor, sensorEV3_Ultrasonic)
#pragma config(Motor, motorA, armMotor, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorB, leftMotor, tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor, motorC, rightMotor, tmotorEV3_Large, PIDControl, driveRight, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

TSemaphore semaphoreEvitarColisiones;
TSemaphore semaphoreDetectarLuz;
TSemaphore semaphoreSeguirparedes;
TSemaphore semaphoreIrrecto;

int luz = 255;
int umbral = 25;
bool inhibitedColision = true;
bool inhibitedLuz = true;
bool inhibitedSeguirParedes = true;
bool inhibitedirrecto = true;


task evitarcolisiones() {


    while (true) {


        semaphoreLock(semaphoreEvitarColisiones);


        if (bDoesTaskOwnSemaphore(semaphoreEvitarColisiones)) {

            if (inhibitedColision) {

                semaphoreUnlock(semaphoreEvitarColisiones);

                // Si se detectó una colisión, escapar
                if (getTouchValue(touchSensor) == 1 ||  getUSDistance(sonarSensor) < 10 ) {

                    semaphoreLock(semaphoreIrrecto);
                    inhibitedirrecto = false;
                    semaphoreUnlock(semaphoreIrrecto);

                    clearTimer(T1);
                    while (time10[T1] < 100) {

                        setMotorSpeed(leftMotor, -20);
                        setMotorSpeed(rightMotor, -20);
                    }

                    resetGyro(gyroSensor);
                    setMotorSpeed(leftMotor, 20);
                    setMotorSpeed(rightMotor, -20);
                    while (getGyroDegrees(gyroSensor) < 70) {}


                } else { 

 		    semaphoreLock(semaphoreDetectarLuz);
                    inhibitedLuz = true;
                    semaphoreUnlock(semaphoreDetectarLuz);

                }

            } else {

                 semaphoreUnlock(semaphoreEvitarColisiones);
               semaphoreLock(semaphoreDetectarLuz);
                    inhibitedLuz = false;
                    semaphoreUnlock(semaphoreDetectarLuz);
            }


        }


    }

}





task detectarluz() {


    while (true) {

        semaphoreLock(semaphoreDetectarLuz);

        if (bDoesTaskOwnSemaphore(semaphoreDetectarLuz)) {

            if (inhibitedLuz) {

                semaphoreUnlock(semaphoreDetectarLuz);

                if (getColorAmbient(colorSensor) > 10) {

                    semaphoreLock(semaphoreSeguirparedes);
                    inhibitedSeguirParedes = false;
                    semaphoreUnlock(semaphoreSeguirparedes);

                    while (getColorAmbient(colorSensor) <= luz) {
                        luz = getColorAmbient(colorSensor);

                        setMotorSpeed(leftMotor, 20);
                        setMotorSpeed(rightMotor, -20);

                        if(getColorAmbient(colorSensor) >= luz){


                          setMotorSpeed(leftMotor, -80);
                        setMotorSpeed(rightMotor, 80);

                      }


                    }

                    setMotorSpeed(leftMotor, 20);
                    setMotorSpeed(rightMotor, 20);


                    if (getColorAmbient(colorSensor) > umbral) {
                        setMotorSpeed(leftMotor, 0);
                        setMotorSpeed(rightMotor, 0);
                    }


                } else {
                    semaphoreLock(semaphoreSeguirparedes);
                    inhibitedSeguirParedes = true;
                    semaphoreUnlock(semaphoreSeguirparedes);
                }

            } else {


                semaphoreUnlock(semaphoreDetectarLuz);
                semaphoreLock(semaphoreSeguirparedes);
                inhibitedSeguirParedes = false;
                semaphoreUnlock(semaphoreSeguirparedes);

            }

        }
    }
}







task seguirparedes() {


    int min = 255;
    int current = 0;
    int detectar = 0;


    while (true) {

        semaphoreLock(semaphoreSeguirparedes);


        if (bDoesTaskOwnSemaphore(semaphoreSeguirparedes)) {

            if (inhibitedSeguirParedes) {


                semaphoreUnlock(semaphoreSeguirparedes);

                if (getUSDistance(sonarSensor) < 25) {

                    semaphoreLock(semaphoreIrrecto);
                    inhibitedirrecto = false;
                    semaphoreUnlock(semaphoreIrrecto);


                    if (detectar == 0) {

                        min = 255;
                        current = getUSDistance(sonarSensor);

                        while (getUSDistance(sonarSensor) <= min) {
                            min = getUSDistance(sonarSensor);
                            setMotorSpeed(leftMotor, 20);
                            setMotorSpeed(rightMotor, -20);


                            if(getUSDistance(sonarSensor)>= min){

                          setMotorSpeed(leftMotor, -20);
                        setMotorSpeed(rightMotor, 20);


                      }


                        }

                        detectar = 1;
                    }


                    resetGyro(gyroSensor);
                    setMotorSpeed(leftMotor, 20);
                    setMotorSpeed(rightMotor, -20);
                    while (getGyroDegrees(gyroSensor) < 70) {}

                    clearTimer(T1);
                    while (time10[T1] < 200) {
                        setMotorSpeed(leftMotor, 20);
                        setMotorSpeed(rightMotor, 20);

                        if(getUSDistance(sonarSensor)<= 15  ){

                    resetGyro(gyroSensor);
                    setMotorSpeed(leftMotor, 20);
                    setMotorSpeed(rightMotor, -20);

                    while (getGyroDegrees(gyroSensor) < 70) {}

                    }


                    }

                    resetGyro(gyroSensor);
                    setMotorSpeed(leftMotor, -30);
                    setMotorSpeed(rightMotor, 30);

                    while (getGyroDegrees(gyroSensor) > -60) {}


                    if (getUSDistance(sonarSensor) > 25) {

                        setMotorSpeed(leftMotor, 50);
                        setMotorSpeed(rightMotor, 50);


                    }

                    if (getUSDistance(sonarSensor) < 20) {
                        setMotorSpeed(leftMotor, -50);
                        setMotorSpeed(rightMotor, -50);

                    }


                } else {

                    semaphoreLock(semaphoreIrrecto);
                    inhibitedirrecto = true;
                    semaphoreUnlock(semaphoreIrrecto);

                }


            } else {

                 semaphoreUnlock(semaphoreSeguirparedes);
                semaphoreLock(semaphoreIrrecto);
                inhibitedirrecto = false;
                semaphoreUnlock(semaphoreIrrecto);


            }


        }


    }

}


task irrecto() {

    while (true) {


        semaphoreLock(semaphoreIrrecto);

        if (bDoesTaskOwnSemaphore(semaphoreIrrecto)) {

            if (inhibitedirrecto) {

                if (getUSDistance(sonarSensor) > 25) {
			
                    setMotorSpeed(leftMotor, 30);
                    setMotorSpeed(rightMotor, 30);

                }
            }
        }

      semaphoreUnlock(semaphoreIrrecto);

    }

}


task main() {

	//moveMotor(armMotor, 70, degrees, 20);
   semaphoreInitialize(semaphoreEvitarColisiones);
    semaphoreInitialize(semaphoreDetectarLuz);
    semaphoreInitialize(semaphoreSeguirparedes);
    semaphoreInitialize(semaphoreIrrecto);

    startTask(evitarcolisiones);
    startTask(detectarluz);
   startTask(seguirparedes);
    startTask(irrecto);

    while (true) {
        abortTimeslice();
    }

}
