#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

//inicializamos variables para la comunicacion entre ros2 y microros
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_timer_t timer;

//variables para el calculo de la velocidad con el encoder
int32_t tiempo_act = 0, tiempo_ant = 0, delta_tiempo = 2e9; // Variables para calcular el delta de tiempo
int pwm = 0;                                                // valor de salida al PWM
char opcion;                                                // Letra leida del puerto serial para la direccion del motor
float posicion=0, posactual = 0, posanterior = 0, velocidad = 0;// Posiciones del encoder
float resolucion = 0.0865;                                  // Definir resolución del encoder
int pulsos = 4160;                                          // Número de pulsos a la salida del motorreductor
int32_t contador = 0, contaux = 0, revoluciones;            // Conteo de los pulsos del encoder
volatile bool BSet = 0;                                     // 
volatile bool ASet = 0;                                     //
volatile bool encoderDirection = false;

//variables para el caluclo del PID
float KP = 20;
float KD = 2;
float KI = 5;
float TS = .02;
float GN = 0;
float pruv = 0;

//variables para el calculo del error
float med_err = 0;
float error = 0;
float error_prev = 0;
float setpoint_value = 0;

//definimos los pines que utilizaremos
#define LED_PIN 13
#define ENA_PH 12
#define N3_PH 32
#define N4_PH 27
#define ENC_A 25
#define ENC_B 26

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Funcion para errores relacionados con la comunicacion entre ros2 y microros
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void IRAM_ATTR Encoder()
{
  BSet = digitalRead(ENC_B);
  ASet = digitalRead(ENC_A);
  //Si ambas señales leidas son iguales, el motor gira en sentido anti-horario
  //y se incrementa un contador para saber el número de lecturas
  if (BSet == ASet)
  {
    contador++;
    encoderDirection = true;
  }
  //Si ambas señales leídas son distintas, el motor gira en sentido horario
  //y se decrementa un contador para saber el número de lecturas
  else
  {
    contador--;
    encoderDirection = false;
  }
  //Calcular delta de tiempo
  tiempo_act = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant = tiempo_act;
}

//Funcion para obtener la posicion del encoder
void pose ()
{
  if (encoderDirection)
  {
    posicion = contador * resolucion; //Convertir a grados
    if (contador >= pulsos) //Contar por revoluciones
    {
      revoluciones++;
      contador = 0;
    }
  }
  else
  {
    posicion = contador * resolucion; //Convertir a grados 
    if (contador <= -pulsos) //Contar por revoluciones
    {
      revoluciones--;
      contador = 0;
    }
  }
  //Cálculo de la velocidad mediante un delta de tiempo  
  velocidad = 60000000/(pulsos * delta_tiempo); 
  med_err = map(velocidad,0,46,0,255);
  //Se pregunta por la velocidad, cuando hay una inversion de giro, para hacerla positiva
  if (velocidad < 0)
    velocidad = abs(velocidad);
  encoderDirection = false; // Reiniciar la variable después de usarla
  
}

//Funcion para la implementacion del error, PID y pwm 
void PWM_int(float spv){
  //obtenemos el dato que nos manda ros 2 y le implementamos un pwm sensillo
  int pwm = int(spv * 255);
  //tomamos las lecturas del error y obtenemos el error
  error = pwm - med_err;
  //implementamos PID
  GN = KP*error+(KD/TS)*(error-error_prev)+(KI*TS)*(error+error_prev);
  //asignamos el valor actual de error a error_prev
  error_prev = error;
  //mandamos el dato dependiendo si es positio o negativo para cambiar la direccion
  if (GN >= 0 && GN <= 255) {
    digitalWrite(N3_PH, HIGH);
    digitalWrite(N4_PH, LOW); 
    analogWrite(ENA_PH, abs(GN));
  }
  else if (GN >= -255 && GN <= 0) {
    digitalWrite(N3_PH, LOW);
    digitalWrite(N4_PH, HIGH); 
    analogWrite(ENA_PH, abs(GN)); 
  }
}

//funciones para la comunicacion 
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  setpoint_value = msg->data;
  PWM_int(setpoint_value); // Corrección: Asegurar que PWM_int esté correctamente definido
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //int pwm = int(setpoint_value * 255);
    msg.data = setpoint_value;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  Serial.begin(9600);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA_PH, OUTPUT);
  pinMode(N3_PH, OUTPUT);
  pinMode(N4_PH, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  digitalWrite(LED_PIN, HIGH);

  //delay(2000);

  allocator = rcl_get_default_allocator();
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "Azul_Controler", "", &support));

  
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "Azul_Setpoint"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "Azul_Controler"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "Azul_Setpoint"));
  
  const unsigned int timer_timeout = .01;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}


void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // Aquí puedes implementar la lógica para utilizar `setpoint_value` con tu controlador motor
}
