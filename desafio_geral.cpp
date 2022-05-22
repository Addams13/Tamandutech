#include <stdint.h>


// ----------------------------------------------------- //
// PARTE 1 DO DESAFIO GERAL - ORGANIZAÇÃO DOS DADOS //

#define NUM_SENS 5 // assumindo que existem 5 sensores.

// PARAMETROS DA VELOCIDADE

// As velocidades max e min são iguais para as rodas esq. e dir.
// por isso se definiu um valor universal para cada, usando valores aleatórios
#define MAX_SPEED 255
#define MIN_SPEED 0

typedef struct valueSpeed{ // PARAMETROS DA VELOCIDADE
    struct {
        int base;
    }curve, strait;
};

typedef struct valuesPID{ // PARAMETROS DO PID
    struct {
        float setpoint;
        float Kp;
        float Ki;
        float Kd;
    }curve, strait;
};

// PARAMETROS E VALORES DOS SENSORES ARRAY E LATERAIS
// NO MESMO STRUCT POR  SIMPLICIDADE
typedef struct sensores{
    struct {
        int max_value;
        int min_value;
        int8_t valueEach [NUM_SENS];
        int8_t mean;
    }Array, Lat;
};


// VARIAVEIS DAS MARCAS LATERAIS E DOS ENCODER MOTORES
typedef struct quant{
    int right_quant;
    int left_quant;
};

// JUNTANDO TODOS OS VALORES NO ROBO
typedef struct valueCar {
    int8_t state; // igual ao ex.: 0 = parado; 1 = linha; 2 = curva;
    valueSpeed rMotSpeed;
    valueSpeed lMotSpeed;
    quant latMarks;
    quant motEncs;
    valuesPID PID;
    sensores s;
    uint32_t lastUpdate;

    valueCar(): state(1), lastUpdate(0) {};
};


// ----------------------------------------------------- //
// PARTE 2 DO DESAFIO GERAL - OTIMIZAÇÃO //

valueCar car; 
float previus_error;
float I; // Termo integral do PID, cumulativo

void vTaskCalcularPID(float *input, float *output){
    // Considerei que o input é igual a PV (process variable),
    // ou seja, a leitura atual do sensor.
    // Como o erro é SP - PV (SP = setpoint) deixei as variáveis do segue-linha como globais
    // para acessar os parâmetros do PID.

    // Output é o PID utilizado para modificar a velocidade os motores.
    if(car.state == 1){
        float error = car.PID.strait.setpoint - *input;

        *output = (car.PID.strait.Kp*error)+(car.PID.strait.Ki*(I+error))+(car.PID.strait.Kd*(error-previus_error));
        previus_error = error;
    }else if(car.state == 2){
        float error = car.PID.curve.setpoint - *input;

        *output = (car.PID.curve.Kp*error)+(car.PID.curve.Ki*(I+error))+(car.PID.curve.Kd*(error-previus_error));
        previus_error = error;
    }
    
};

int main(){
    return 0;
}