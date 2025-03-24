#include "dlab_def.h"
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

#define MAXS 10000 

// Declare global variables
pthread_t Control;  // Control thread
sem_t data_avail;   // Semaphore

float theta[MAXS];  // Array to store motor positions
float ref[MAXS];    // Array for reference input

float Kp = 65;      // Proportional gain
float Fs = 200.0;    // Sampling frequency 
float run_time = 10; // RUNTIME
int motor_number = 9; // Motor number (CHANGE LATER)

// Function prototype
void *ControlLoop(void *arg);

int main() {
    char selection;

    while (1) {
        // Display menu
        printf("\nMenu:\n");
        printf("r: Run control\n");
        printf("p: Change Kp\n");
        printf("f: Change sampling frequency\n");
        printf("t: Change run time\n");
        printf("u: Select input type Step\n");
        printf("g: Plot response\n");
        printf("h: Save response\n");
        printf("q: Quit\n");
        printf("Enter selection: ");
       
        selection = getchar();
        getchar();  // Consume newline

        switch (selection) {
            case 'r': // Run control
                sem_init(&data_avail, 0, 0);
                Initialize(Fs, motor_number);
                pthread_create(&Control, NULL, ControlLoop, NULL);
                pthread_join(Control, NULL);
                Terminate();
                sem_destroy(&data_avail);
                break;

            case 'p': // Change Kp
                printf("Enter new Kp: ");
                scanf("%f", &Kp);
                getchar();
                break;

            case 'f': // Change sampling frequency
                printf("Enter new Fs: ");
                scanf("%f", &Fs);
                getchar();
                break;

            case 't': // Change run time
                printf("Enter new run time (s): ");
                scanf("%f", &run_time);
                getchar();
                break;

            case 'u': // Choose input type (Step)
                printf("Step input selected. Enter magnitude: ");
                float magnitude;
                scanf("%f", &magnitude);
                getchar();

                for (int i = 0; i < MAXS; i++) {
                    ref[i] = magnitude * 3.14 / 180;  // Basic step response
                }
                break;

            case 'g': // Plot response
                plot(ref, theta, Fs, run_time * Fs, SCREEN, "Motor Position", "Time", "Angle");
                break;

            case 'h': // Save plot
                plot(ref, theta, Fs, run_time * Fs, PS, "Motor Position", "Time", "Angle");
                break;

            case 'q': // Quit program
                return 0;

            default:
                printf("Invalid selection\n");
        }
    }
}

void *ControlLoop(void *arg) {
    int k = 0;
    int no_of_samples = (int)(run_time * Fs);
    float motor_position, error, control_signal;

    while (k < no_of_samples) {
        sem_wait(&data_avail);  // Wait for encoder data

        motor_position = EtoR(ReadEncoder());  // motor position
        error = ref[k] - motor_position;  // error
        control_signal = Kp * error;  // control signal response

        DtoA(VtoD(control_signal)); //D2A

        theta[k] = motor_position;  // storing position data
        k++;
    }

    pthread_exit(NULL);
}
