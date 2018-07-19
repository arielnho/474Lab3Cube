#include <stdint.h>
#include <math.h>
#include "init.c"
#include "ee474.h"
#include "SSD2119.h"

#define FLAG_NONE 0
#define FLAG_ONE 3
#define FLAG_ADC 5

#define RED 0x02
#define BLUE 0x04
#define VIOLET 0x06
#define GREEN 0x08
#define YELLOW 0x0A
#define LIGHT_BLUE 0x0C
#define WHITE 0x0E

#define MHZ16 (unsigned long)0xF42400
#define MHZ80 (unsigned long)0x4C4B400
#define MHZ4 (unsigned long)0x3D0900

#define offsetW 180
#define offsetH 120


typedef struct Node {
  short x, y, z;
} Node;

typedef struct Edge {
  short n1, n2;
} Edge;

typedef struct Nodes {
  Node n[8];
} Nodes;

typedef struct Edges {
  Edge e[1];
} Edges;

Node nodes[8] = {{-18, -18, -18}, {-18, -18, 18}, {-18, 18, -18}, {-18, 18, 18}, {18, -18, -18}, {18, -18, 18}, {18, 18, -18}, {18, 18, 18}};
Edge edges[12] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

unsigned long FLAG = FLAG_NONE;
unsigned long result = 0;
unsigned char output[3];

int SerialFlag = 0;

void LED_ON(unsigned int color);
void LED_OFF(void);
void Switching(void);
void Rotating_Cube(void);
void Rotate_X(short theta);


void transmit(char data){
  while(UART0_FR&0x0020 != 0);
  UART0_DR = data;
}

int recieve(){
  while(UART0_FR&0x0010 != 0);
  return ((int) (UART0_DR&0xFF));
}

void Timer0A_Handler(void) {
  FLAG = FLAG_ONE;
  ADC0_PSSI = 0x008;
  TIMER_RESET0 |= (0x1<<0);
}


void PortF_Handler(void) {
  if (GPIO_WRITE_PORTF == 0x01) {
    ADC0_IM &= ~0x8;
    PLL_Init(4);
    UART_Init(4);
    TIMER_VAL0 = MHZ4;  //set timer start to 4000000
    ADC0_IM |= 0x8;
    LED_ON(RED);
  }
  if (GPIO_WRITE_PORTF == 0x10) {
    ADC0_IM &= ~0x8;
    PLL_Init(80);
    UART_Init(80);
    TIMER_VAL0 = MHZ80;  //set timer start to 4000000
    ADC0_IM |= 0x8;   
  }
  GPIO_ICR_PORTF |= 0x11; // clear the interrupt flag before return
}

void ADC0_Handler(void) {
  FLAG = FLAG_ADC;
  while((ADC0_RIS&0x08)==0){};   // wait for conversion done
  unsigned long ADCvalue = ADC0_SSFIFO3&0xFFF;   // read result
  result = (long) (147.5-((74*(3.3)*ADCvalue)/4096));
  ADC0_ISC = 0x0008;             // acknowledge completion
  ADC0_DCISC |=0x8;
}

int main()
{
  PortF_Init();
  Timer0_Init(MHZ16);
  PLL_Init(16);
  UART_Init(16);
  ADC_Init();
  Interrupt_Init();
  LED_OFF();
  LCD_Init();
  Rotating_Cube();
  return 0;
}

//turn on given color
void LED_ON(unsigned int color){
  GPIO_WRITE_PORTF = 0x00;
  GPIO_WRITE_PORTF |= color;
}

//make PA2 low
void LED_OFF(void) {
  GPIO_WRITE_PORTF = 0x00;
}

void Rotating_Cube(void) {
  while (1) {
    LCD_ColorFill(Color4[15]);
    for (int i = 0; i < 12; i++) {
      Node n1 = nodes[edges[i].n1];
      Node n2 = nodes[edges[i].n2];
      LCD_DrawLine(180 + n1.x, 120 + n1.y, 180 + n2.x, 120 + n2.y, Color4[0]);
    }
    Rotate_X(10);
  }
}

void Rotate_X(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    Node n = nodes[i];
    short y = n.y;
    short z = n.z;
    n.y = (short)(y * cosTheta - z * sinTheta);
    n.z = (short)(z * cosTheta + y * sinTheta);
  }
}

void Switching(void) {
  while (1) {      
    if (result > 0 && result <= 17) {
      LED_ON(RED);
    } else if (result > 17 && result <= 19) {
      LED_ON(BLUE);
    } else if (result > 19 && result <= 21) {
      LED_ON(VIOLET);
    } else if (result > 21 && result <= 23) {
      LED_ON(GREEN);
    } else if (result > 23 && result <= 25) {
      LED_ON(YELLOW);
    } else if (result > 25 && result <= 27) {
      LED_ON(LIGHT_BLUE);
    } else if (result > 27 && result <= 40) {
      LED_ON(WHITE);
    } else {
      LED_OFF();
    }
    
    if(FLAG == FLAG_ADC){
      unsigned short tempResult = (unsigned short)result;
      output[0] = tempResult/10 + 0x30;  // tens digit
      tempResult = tempResult%10;
      output[1] = tempResult + 0x30;     // ones digit
      output[2] = 0;            // null termination
      transmit(output[0]);
      transmit(output[1]);
      FLAG = FLAG_NONE;
    }
  }
}
