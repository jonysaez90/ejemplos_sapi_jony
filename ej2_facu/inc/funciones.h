/**
*	\file funciones.h
*	\brief Resumen del archivo
*	\details Descripcion detallada del archivo
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
//declaracion de variables
int cont;

//Declaracion de estados
//#define TRUE  1
//#define FALSE  0
#define 	INICIAL	0
#define 	ROJO	1
#define 	LED_1	2
#define 	LED_2	3
#define 	LED_3	4

//Prototipos de los eventos
int PRESS(void);
int PRESS_4(void);
int PRESS_7(void);
int PRESS_10(void);

//Prototipos de las acciones
void LED_R(void);
void P_LED_1(void);
void P_LED_2(void);
void P_LED_3(void);
void LED_G(void);
void LED_B(void);
void LED_R(void);
void BORROW(void);
