/* ********************************************************************** */
/*               R.A.V.E.N. Followers Quadroter mbed Code                 */
/*                                                                        */
/* Authors : William Etter (UPenn EE '11)                                 */
/*           Paul Martin (UPenn EE 'll)                                   */
/*                                                                        */
/* Date : April 5, 2011                                                   */
/* Version : 1.3                                                          */
/* Hardware : mbed Microcontroller, Sonar, TPA81 Thermopile Array,        */
/*            Pixart IR Camera                                            */
/* This program is free software: you can redistribute it and/or modify   */
/* it under the terms of the GNU General Public License as published by   */
/* the Free Software Foundation, either version 3 of the License, or      */
/* (at your option) any later version.                                    */
/*                                                                        */
/* This program is distributed in the hope that it will be useful,        */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of         */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the           */
/* GNU General Public License for more details.                           */
/*                                                                        */
/* You should have received a copy of the GNU General Public License      */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.   */
/* ********************************************************************** */


/* ****************************************************************************** */
/* ********************   Configuration Definitions  **************************** */
/* ****************************************************************************** */

/* ****************************************************************************** */
/* ******************************   TO DO  ************************************** */
/* ****************************************************************************** */
//  - Altitude PID
//  - Thermal Sensor
//  - Sonar Sensor
//  - IR Obstacle avoidance

/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */
// mbed General
#include "mbed.h"

#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE 1024
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE 1024 

#include "MODSERIAL.h"
#include "math.h"

// Raven Specific
#include "raven.h"
#include "mbedimu.h"
#include "mbedircam.h"
#include "mbedrccontroller.h"
#include "mbedsonar.h"
#include "mbedthermal.h"
#include "mbedxbee.h"

/* ************************************************************ */
/* **************** MAIN PROGRAM - SETUP ********************** */
/* ************************************************************ */
int main() {
    // enable global interrupts
    __enable_irq();
    
    // System initializations
    sweepLEDs();
    Timer timer;
    uint64_t slow_timer = 0;
    uint64_t medium_timer = 0;
    uint64_t mediumfast_timer = 0;
    uint64_t fast_timer = 0;
    uint64_t fastest_timer = 0;
    timer.start();
    wait(0.5);
    //initThermal();
    initIMU();
    initUSB();
    initIRcam();
    wait(1);
    //initSonar();
    initXbee();
    initRC();
    
    pulseLEDs(2);
    wait(0.5);
    
    /* ************************************************************ */
    /* **************       MAIN PROGRAM LOOP  ******************** */
    /* ************************************************************ */
    while(1) { 
        // --- SLOW LOOP (5) Hz ---
        if((timer.read_ms() - slow_timer) >= 200){
            slow_timer = timer.read_ms();
            led1 = !led1;
            //pc.printf("My Yaw = %f \r\n", MyAttitude.Yaw);
        }
        
        // --- MEDIUM LOOP (20) Hz ---
        if((timer.read_ms() - medium_timer) >= 50){
            medium_timer = timer.read_ms();
            led2 = !led2;
            if(followleader){
                measureIRcam();
            }
            xbeeSendBase();
        }
        
        // --- MEDIUM FAST LOOP (50) Hz ---
        if((timer.read_ms() - mediumfast_timer) >= 20){
            mediumfast_timer = timer.read_ms();
            
            xbeeRead();
        }
        
        // --- FAST LOOP (100 Hz) ---
        if((timer.read_ms() - fast_timer) >= 10){
            fast_timer = timer.read_ms();
            //led3 = !led3;
            if(followleader){
                readIRcam();
            }
        }
        
        // --- FASTEST LOOP (200 Hz) ---
        if((timer.read_ms() - fastest_timer) >= 5){
            fastest_timer = timer.read_ms();
            led3 = !led3;
            IMUread();
        }

        // reset timer if 1000 seconds have elapsed
        if(timer.read() >= 1000){
            timer.reset();
            slow_timer = 0;
            medium_timer = 0;
            fast_timer = 0;
        }
    }
}