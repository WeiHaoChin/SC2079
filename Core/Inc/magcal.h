/*
 * magcal.h
 *
 *  Created on: Feb 2, 2025
 *      Author: user
 */

#ifndef INC_MAGCAL_H_
#define INC_MAGCAL_H_
#include "oled.h"
#include "ICM20948.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"

extern I2C_HandleTypeDef hi2c1;
#define MAGCAL_POINTS 100
float dist_squared(float x1, float x2, float y1, float y2, float z1, float z2);
typedef struct {
	float offset_HI[3];
	float matrix_SI[3][3];
} MagCalParams;

 //MagCalParams *params;
 void magcal_calc_params(MagCalParams *params) {
     float magRange[3][2]; // minimum, maximum for each axis (X, Y, Z)
     float mag[3];         // X, Y, Z readings
     char buf[20];

     // Load initial values
     HAL_Delay(500);
     ICM20948_readMagnetometer_all(&hi2c1, mag);  // Read X, Y, Z values
     magRange[0][0] = mag[0];
     magRange[0][1] = mag[0];
     magRange[1][0] = mag[1];
     magRange[1][1] = mag[1];
     magRange[2][0] = mag[2];
     magRange[2][1] = mag[2];

     // Find minimum and maximum for X, Y, Z axes
     OLED_ShowString(0, 0, "Min/Max XYZ");
     uint16_t i = 0;
     while (1) {
         ICM20948_readMagnetometer_all(&hi2c1, mag);
         snprintf(buf, 20, "%.2f|%.2f|%.2f", mag[0], mag[1], mag[2]);
         OLED_ShowString(0, 10, buf);

         for (i = 0; i < 3; i++) {
             if (mag[i] < magRange[i][0]) magRange[i][0] = mag[i];
             if (mag[i] > magRange[i][1]) magRange[i][1] = mag[i];

             snprintf(buf, 20, "%.3f|%.3f", magRange[i][0], magRange[i][1]);
             OLED_ShowString(0, 10 * (i + 2), buf);
         }

         OLED_Refresh_Gram();
         if (user_is_pressed()) break;
     }

     // Hard iron offset (center of the ellipsoid).
     params->offset_HI[0] = (magRange[0][1] + magRange[0][0]) / 2;
     params->offset_HI[1] = (magRange[1][1] + magRange[1][0]) / 2;
     params->offset_HI[2] = (magRange[2][1] + magRange[2][0]) / 2;

     // Calculate step size for each axis (to get an even number of readings)
     float xStep = (magRange[0][1] - magRange[0][0]) / MAGCAL_POINTS;
     float yStep = (magRange[1][1] - magRange[1][0]) / MAGCAL_POINTS;
     float zStep = (magRange[2][1] - magRange[2][0]) / MAGCAL_POINTS;

     // Read ellipse points
     i = 0;
     float magVals[3][MAGCAL_POINTS];

     OLED_Clear();
     OLED_ShowString(0, 0, "Tracing ellipse");
     snprintf(buf, 20, "%.2f|%.2f|%.2f", xStep, yStep, zStep);
     OLED_ShowString(0, 20, buf);
     OLED_Refresh_Gram();

     // Trace ellipse (and also get b)
     float b = -1, dist;
     while (i < MAGCAL_POINTS) {
         ICM20948_readMagnetometer_all(&hi2c1, mag);
         if (i == 0 || (fabs(mag[0] - magVals[0][i-1]) > xStep &&
                         fabs(mag[1] - magVals[1][i-1]) > yStep &&
                         fabs(mag[2] - magVals[2][i-1]) > zStep)) {
             magVals[0][i] = mag[0];
             magVals[1][i] = mag[1];
             magVals[2][i] = mag[2];

             snprintf(buf, 40, "On %i of %i", i+1, MAGCAL_POINTS);
             OLED_ShowString(0, 10, buf);
             OLED_Refresh_Gram();
             i++;
         }

         dist = dist_squared(params->offset_HI[0], mag[0], params->offset_HI[1], mag[1], params->offset_HI[2], mag[2]);
         if (b < 0 || dist < b) {
             b = dist;
         }
     }

     b = (float) sqrt((double) b);
     // Find major axis points on the ellipse
     uint16_t j = 0;
     uint16_t a1, a2;
     float a = 0;
     for (i = 0; i < MAGCAL_POINTS - 1; i++) {
         for (j = i + 1; j < MAGCAL_POINTS; j++) {
             dist = dist_squared(magVals[0][i], magVals[0][j], magVals[1][j], magVals[1][j], magVals[2][j], magVals[2][j]);
             if (dist > a) {
                 a = dist;
                 a1 = i; a2 = j;
             }
         }
     }

     // Calculate required parameters
     a = (float) sqrt((double) a) / 2;

     // Re-using mag as [k1, k2, k3]
     mag[0] = fabs(magVals[1][a1] - params->offset_HI[1]) / a;
     mag[1] = fabs(magVals[0][a1] - params->offset_HI[0]) / a;
     mag[2] = fabs(magVals[2][a1] - params->offset_HI[2]) / a;

     // Check rotation (clockwise or counter-clockwise)
     i = (magVals[0][a1] > params->offset_HI[0] && magVals[1][a1] > params->offset_HI[1] && magVals[2][a1] > params->offset_HI[2]) ||
         (magVals[0][a2] > params->offset_HI[0] && magVals[1][a2] > params->offset_HI[1] && magVals[2][a2] > params->offset_HI[2]);

     // Soft iron matrix (3x3)
     params->matrix_SI[0][0] = mag[1] * a / b;
     params->matrix_SI[0][1] = mag[0] * a / b;
     params->matrix_SI[0][2] = 0; // Assuming no cross-axis terms for Z in the X-Y plane.
     params->matrix_SI[1][0] = -(mag[0]) * a / b;
     params->matrix_SI[1][1] = mag[1] * a / b;
     params->matrix_SI[1][2] = 0; // Assuming no cross-axis terms for Z in the X-Y plane.
     params->matrix_SI[2][0] = 0;
     params->matrix_SI[2][1] = 0;
     params->matrix_SI[2][2] = mag[2] * a / b;  // For the Z-axis, you need to apply the same scaling.


     if (!i) {
         // Flip if counter-clockwise
         params->matrix_SI[0][1] = -(params->matrix_SI[0][1]);
         params->matrix_SI[1][0] = -(params->matrix_SI[1][0]);
         params->matrix_SI[1][2] = -(params->matrix_SI[1][2]);
     }

     // User indication
     OLED_Clear();
     OLED_ShowString(0, 0, "MagCal done");
     snprintf(buf, 20, "%.2f %.2f %.2f", params->offset_HI[0], params->offset_HI[1], params->offset_HI[2]);
     OLED_ShowString(0, 10, buf);
     snprintf(buf, 20, "%.2f %.2f %.2f", params->matrix_SI[0][0], params->matrix_SI[0][1], params->matrix_SI[0][2]);
     OLED_ShowString(0, 20, buf);
     snprintf(buf, 20, "%.2f %.2f %.2f", params->matrix_SI[1][0], params->matrix_SI[1][1], params->matrix_SI[1][2]);
     OLED_ShowString(0, 30, buf);
     snprintf(buf, 20, "%.2f %.2f %.2f", params->matrix_SI[2][0], params->matrix_SI[2][1], params->matrix_SI[2][2]);
     OLED_ShowString(0, 40, buf);
     OLED_Refresh_Gram();

     while (!user_is_pressed());
     OLED_Clear();
     OLED_Refresh_Gram();
 }

int user_is_pressed(){ return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==0;}

float dist_squared(float x1, float x2, float y1, float y2, float z1, float z2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return dx*dx + dy*dy + dz*dz;
}

void magcal_adjust(float magXYZ[3],MagCalParams *params) {
    float x = magXYZ[0] - params->offset_HI[0];
    float y = magXYZ[1] - params->offset_HI[1];
    float z = magXYZ[2] - params->offset_HI[2];

    magXYZ[0]=x;
    magXYZ[1]=y;
    magXYZ[2]=z;
    magXYZ[0] = params->matrix_SI[0][0] * x + params->matrix_SI[0][1] * y + params->matrix_SI[0][2] * z;
    magXYZ[1] = params->matrix_SI[1][0] * x + params->matrix_SI[1][1] * y + params->matrix_SI[1][2] * z;
    magXYZ[2] = params->matrix_SI[2][0] * x + params->matrix_SI[2][1] * y + params->matrix_SI[2][2] * z;
}


#endif /* INC_MAGCAL_H_ */
