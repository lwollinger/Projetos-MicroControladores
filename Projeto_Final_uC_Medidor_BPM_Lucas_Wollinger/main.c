// Lucas Wollinger - IFSC Florianopolis - MicroControladores 1

/*
 * O projeto visa desenvolver uma ferramenta que consiga armazenar os valores dos picos
 * de tensão gerados pelo eletrocardiograma (ECG), a partir desta observar os tempos dos picos
 * fazer os calculos da frequencia cardiaca e mostrar em um display gráfico a onda do ECG e a
 * frequencia cardiaca.
 */

#include <msp430.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include "st7735.h"

/*
 * Valores de 0 - 9 para desenhar no display
 */
static const uint8_t Font[] = {
  0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
  0x00, 0x42, 0x7F, 0x40, 0x00, // 1
  0x72, 0x49, 0x49, 0x49, 0x46, // 2
  0x21, 0x41, 0x49, 0x4D, 0x33, // 3
  0x18, 0x14, 0x12, 0x7F, 0x10, // 4
  0x27, 0x45, 0x45, 0x45, 0x39, // 5
  0x3C, 0x4A, 0x49, 0x49, 0x31, // 6
  0x41, 0x21, 0x11, 0x09, 0x07, // 7
  0x36, 0x49, 0x49, 0x49, 0x36, // 8
  0x46, 0x49, 0x49, 0x29, 0x1E, // 9
};

volatile unsigned int Tensao_ref = 0;                   // Variável para tensão de referencia (soma de 16 pontos).
volatile unsigned int threshold = 0;                    // Variável para valor limite para buscar os picos--> (Tensão_fixa + Tensao_med).
volatile unsigned int Tensao_med = 0;                   // Variável para calcular média móvel.
volatile unsigned int m = 0;                            // Variável média móvel.

volatile unsigned int Data[100];
volatile unsigned int count = 0;                        // Para controle.
volatile unsigned int Display = 0;                      // Para controle.
volatile unsigned int DataMAX = 0;                      // Valor maximo de tensï¿½o 1 medido pelo sensor.
unsigned char flag = 0;                                 // Indica qual period T1 ou T2 eu vou guardar.
volatile unsigned int time = 0;                         // Contador sem parar.
volatile unsigned int T = 0;                            // Periodo para contagem em int
volatile unsigned int T_seconds = 0;                    // Periodo para conversï¿½o em segundos
volatile unsigned int count1 = 0;
volatile unsigned int count2 = 0;
volatile unsigned int freq_cardiaca = 0;
volatile unsigned char flag_main = 0;
char buffer[20];                                        // Buffer para armazenar a string formatada
volatile unsigned int centena = 0;                      // Variável para calculo da centena do valor da frequencia.
volatile unsigned int dezena = 0;                       // Variável para calculo da dezena do valor da frequencia.
volatile unsigned int unidade = 0;                      // Variável para calculo da unidade do valor da frequencia.
volatile unsigned char i = 0;                           // Variável de controle contador.
volatile unsigned int valor = 0;                        // Variável usada dentro da main para não perder a frequencia cardiaca quando retirar Cent, Dezen, Unid.

#define MIN_INTERVAL 30                                 // 300ms se cada count for 10ms.


/*
 * Definições de pinos do display para o MSP430F5529
 */
#define CS_PIN     BIT0                                 // Chip Select          // P2.0
#define DC_PIN     BIT4                                 // Data/Command // A0   // P2.4
#define RST_PIN    BIT2                                 // Reset                // P2.2
#define MOSI_PIN   BIT0                                 // MOSI (UCB0SIMO)      // P3.0
#define SCLK_PIN   BIT2                                 // SCLK (UCB0CLK)       // P3.2
#define LED        BIT1                                 // BackLight            // P3.1

#define SCREEN_WIDTH  128                               // Tamanho da tela.
#define SCREEN_HEIGHT 160                               // Tamanho da tela.

/*
 * Funções do display gráfico
 */
void ST7735_init(void);
void ST7735_sendData(uint8_t data);
void ST7735_sendCommand(uint8_t command);
void drawPixel(uint8_t x, uint8_t y, uint16_t color);
void fillScreen(uint16_t color);
void clear_front(uint8_t y, uint16_t color);
void ST7735_DrawChar(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size);


/*
 * Configuração do SPI
 */
void SPI_init(void) {
    P3SEL |= MOSI_PIN + SCLK_PIN;
    P3DIR |= MOSI_PIN + SCLK_PIN + LED;
    UCB0CTL1 |= UCSWRST;                                // Coloca o módulo em reset
    UCB0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;        // Fase de clock, MSB primeiro, mestre, síncrono
    UCB0CTL1 |= UCSSEL_2;                               // Seleciona SMCLK como fonte de clock
    UCB0BR0 = 0x00;                                     // Divide o clock (SMCLK / 1)
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;                               // Sai do reset
}




/*         PinOut's Heart Monitor
 *  ____________________    _______________
 * |    Heart Monitor   |  |      MSP430   |
 * |               Gnd <|--|> Gnd          |
 * |               Vcc <|--|> 3,3V         |
 * |            OutPut <|--|> P6.0         |
 * |____________________|  |_______________|
 */



int main(void)
{

    WDTCTL = WDTPW | WDTHOLD;                           // Stop WDT


    /*
     * Configuraçãodo Timer A para Base de Tempo para Coleta de Dados
     */
    TA0CTL = TASSEL_2 | MC_1 | TACLR;
    /* TASSEL_2 ->  Timer A clock source select: 2 - SMCLK
     * MC_1 ->  Timer A mode control: 1 - Up Mode
     * TACLR ->  Timer A counter clear
     * Configuração da fonte do clock do timer 1 */

    TA0CCR0 = 5000;                                     // 5ms de base a cada count, (5ms para entrar na interrupção do timer).
    TA0CCTL0 = CCIE;                                    // CCR0 interrupt enabled --> sermpre que voltar a zero entra no tratamento



    /*
     * Configuração Conversor AD
     */
    P6SEL |= BIT0;                                      // P6.0 ADC option select
    ADC12CTL0 = ADC12SHT02 + ADC12ON;                   // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0;               // Use sampling timer
    ADC12MCTL0 = ADC12INCH_0;                           // ref+=AVcc, channel = A0
    ADC12IE = BIT0;                                     // Enable interrupt
    ADC12CTL0 |= ADC12ENC;



    // Variáveis, Cores e inicialização do Display
    unsigned int j = 0;                                 // controle de seguimento de pixel do display
    SPI_init();                                         // Inicializa o SPI
    ST7735_init();                                      // Inicializa o display
    P3OUT |= 0x02;                                      // Backlight LCD on
    static const uint16_t bk_color = 0x0000;            // Cor preta de fundo
    static const uint16_t sig_color = 0xF81F;           // Cor sinal rosa
    fillScreen(bk_color);


    /*
     * Configurando valores na UART, caso deseja usar.
     */
//    P4SEL |= BIT5+BIT4;                                 // P4.4,5 = USCI_A1 TXD/RXD
//    UCA1CTL1 |= UCSWRST;                                // **Put state machine in reset**
//    UCA1CTL1 |= UCSSEL_2;                               // SMCLK
//    UCA1BR0 = 9;                                        // 1MHz 115200 (see User's Guide)
//    UCA1BR1 = 0;                                        // 1MHz 115200
//    UCA1MCTL |= UCBRS_1 + UCBRF_0;                      // Modulation UCBRSx=1, UCBRFx=0
//    UCA1CTL1 &= ~UCSWRST;                               // **Initialize USCI state machine**



    /*
     * Ativando Interrupções globais
     */
    __bis_SR_register(GIE);                             // Enter LPM0, interrupts enabled
    __no_operation();                                   // For debugger



    while (1) {
        if (flag_main == 1) {
            freq_cardiaca = 12000/(T);                  // Calculo do BPM (60seg / (T * 50 ms))
            flag_main = 0;
            i++;

            // A cada 5 passagens na main calculando a frequencia cardiaca, atualiza o valor no display.
            if(i == 5){
                valor = freq_cardiaca;

                // Condição de controle para valor de centena
                if (valor >= 100) {
                    centena = valor / 100;
                } else {
                    centena = 0;
                }
                ST7735_DrawChar(90, 10, '0' + centena, sig_color, bk_color, 1);
                dezena = (valor % 100) / 10;
                ST7735_DrawChar(97, 10, '0' + dezena, sig_color, bk_color, 1);
                unidade = valor % 10;
                ST7735_DrawChar(105, 10, '0' + unidade, sig_color, bk_color, 1);

                i = 0;
            }
        }

        if(Display == 1){                               // Condição de controle para mostrar display
            Display = 0;
            clear_front(j, bk_color);                   //limpa a coluna
            drawPixel(Data[count]>>5, j, sig_color);
            j++;
            if(j>160)
                j=0;

            /*
             * Caso deseje usar a Uart para teste só retirar o display e retirar comentário abaixo
             */
//            UCA1TXBUF = ADC12MEM0>>8;
//            while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
//            UCA1TXBUF = ADC12MEM0;
//            while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
//
//            UCA1TXBUF = freq_cardiaca>>8;
//            while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
//            UCA1TXBUF = freq_cardiaca;
//            while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
//
//            UCA1TXBUF = DataMAX>>8;
//            while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
//            UCA1TXBUF = DataMAX;
//            while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
        }
    }
}


/*
 * Rotina de tratamento do interruptor TimerA0
 */

// Timer0 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    ADC12CTL0 |= ADC12SC;                                       // Inicia conversao AD
}



// Rotina tratamento da interrupï¿½ï¿½o do conversor AD
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADC12IV,34))
    {
    case  0: break;                           // Vector  0:  No interrupt
    case  2: break;                           // Vector  2:  ADC overflow
    case  4: break;                           // Vector  4:  ADC timing overflow
    case  6:                                  // Vector  6:  ADC12IFG0

        Display = 1;
        Data[count] = ADC12MEM0;                                  // Coleta de Dado no instante tempo count.


        /*
         * Média Móvel para valor limiar do threshold para buscar os picos, mesmo com variação de amplitude (feita pela respiração)
         */
        m++;
        Tensao_ref = Tensao_ref + Data[count];                    // Calcula a média das últimas -- amostras.

        if(m==8)
        {
            m=0;
            Tensao_med = (Tensao_ref>>3);                         // Dividindo valor por 8 para pegar média móvel.
            threshold = 500 + Tensao_med;
            Tensao_ref = 0;
        }



        /* Explicação dos componentes de busca de picos
         *
         * Data[count] > Data[count - 1] --> Evitar ruidos, porque se a minha amostra anterior for menor que descida do sinal não é valido.
         * time - count1 > MIN_INTERVAL --> Garante que o novo pico detectado não ocorra muito cedo após o anterior.
         * threshold --> valor limiar variável para buscar os picos
         */


        // Busca dos Picos de Tensão
        if(flag == 0 && (Data[count] >= threshold)){
            if ((Data[count] > DataMAX) && (Data[count] > Data[count - 1]) && (time - count1 > MIN_INTERVAL)) {
                DataMAX = Data[count];                              // Atualiza o maior valor.
                count1 = time;
            }
        }
        // Busca dos Picos de Tensão
        if(flag == 1 && (Data[count] >= threshold)){
            if((Data[count] > DataMAX) && (Data[count] > Data[count - 1]) && (time - count1 > MIN_INTERVAL)){
                DataMAX = Data[count];                              // Atualiza o maior valor T2.
                count2 = time;
            }
        }



        /*
         * Condições de tratamento de tempo, após encontrar os valores de tempo nos picos
         */
        if (count == 50 && flag == 0) {
            count = 0;
            flag = 1;                                               // Inverte flag.
            DataMAX = 0;
        }

        if (count == 50 && flag == 1) {
            if (count2 > count1) {
                T = count2 - count1;
            }
            // Teste para visualizar se tem erro --> DEBUG!!!!!!!!!!
            if (count2 < count1) {
                UCA1TXBUF = 0xEE;                                   // vai acusar nivel alto.
            }

            count1 = count2;                                        // Busca apenas o proximo valor de count2 e count1 vai ser sempre o antigo count2.
            // reseta as variaveis de controle
            count = 0;
            flag_main = 1;
            DataMAX = 0;
        }

        count++;                                                    // Avança proxima posição do vetor.
        time++;                                                     // Avança variavel de tempo.

        break;

    case  8: break;                           // Vector  8:  ADC12IFG1
    case 10: break;                           // Vector 10:  ADC12IFG2
    case 12: break;                           // Vector 12:  ADC12IFG3
    case 14: break;                           // Vector 14:  ADC12IFG4
    case 16: break;                           // Vector 16:  ADC12IFG5
    case 18: break;                           // Vector 18:  ADC12IFG6
    case 20: break;                           // Vector 20:  ADC12IFG7
    case 22: break;                           // Vector 22:  ADC12IFG8
    case 24: break;                           // Vector 24:  ADC12IFG9
    case 26: break;                           // Vector 26:  ADC12IFG10
    case 28: break;                           // Vector 28:  ADC12IFG11
    case 30: break;                           // Vector 30:  ADC12IFG12
    case 32: break;                           // Vector 32:  ADC12IFG13
    case 34: break;                           // Vector 34:  ADC12IFG14
    default: break;
    }
}


/*
 * Funções para display
 */
void clear_front(uint8_t y, uint16_t color)
{
    uint16_t k,p = 0;

    p = 1;  //Número de colunas a serem limpas entre o sinal antigo e o novo

    P2OUT &= ~CS_PIN;

    ST7735_sendCommand(ST7735_CASET);  // Column address set
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);             // Start column
    ST7735_sendData(0x00);
    ST7735_sendData(SCREEN_WIDTH - 1); // End column

    ST7735_sendCommand(ST7735_RASET);  // Row address set
    ST7735_sendData(0x00);
    ST7735_sendData(y);             // Start row
    ST7735_sendData(0x00);
    ST7735_sendData(y + p);// End row

    ST7735_sendCommand(ST7735_RAMWR);  // Write to RAM

    for (k = 0; k < SCREEN_WIDTH*p; k++)
    {
        ST7735_sendData(color >> 8);
        ST7735_sendData(color & 0xFF);
    }

    P2OUT |= CS_PIN;

}

void fillScreen(uint16_t color)
{
    uint16_t k = 0;

    P2OUT &= ~CS_PIN;

    ST7735_sendCommand(ST7735_CASET);  // Column address set
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);             // Start column
    ST7735_sendData(0x00);
    ST7735_sendData(SCREEN_WIDTH - 1); // End column

    ST7735_sendCommand(ST7735_RASET);  // Row address set
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);             // Start row
    ST7735_sendData(0x00);
    ST7735_sendData(SCREEN_HEIGHT - 1);// End row

    ST7735_sendCommand(ST7735_RAMWR);  // Write to RAM

    for (k = 0; k < SCREEN_WIDTH*SCREEN_HEIGHT; k++)
    {
        ST7735_sendData(color >> 8);
        ST7735_sendData(color & 0xFF);
    }

    P2OUT |= CS_PIN;
}


void drawPixel(uint8_t x, uint8_t y, uint16_t color)
{
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) return;

    P2OUT &= ~CS_PIN;
    ST7735_sendCommand(ST7735_CASET);  // Column address set
    ST7735_sendData(0x00);
    ST7735_sendData(x);                // Start column
    ST7735_sendData(0x00);
    ST7735_sendData(x);                // End column

    ST7735_sendCommand(ST7735_RASET);  // Row address set
    ST7735_sendData(0x00);
    ST7735_sendData(y);                // Start row
    ST7735_sendData(0x00);
    ST7735_sendData(y);                // End row

    ST7735_sendCommand(ST7735_RAMWR);  // Write to RAM
    ST7735_sendData(color >> 8);
    ST7735_sendData(color);
    P2OUT |= CS_PIN;
}


void ST7735_init(void) {
    // Configuração dos pinos de controle
    P2DIR |= CS_PIN + DC_PIN + RST_PIN;
    P2OUT |= CS_PIN;
    P2OUT &= ~RST_PIN;
    __delay_cycles(50000);
    P2OUT |= RST_PIN;

    // Inicialização do display
    P2OUT &= ~CS_PIN;

    ST7735_sendCommand(ST7735_SWRESET); // Software reset
    __delay_cycles(150000);

    ST7735_sendCommand(ST7735_SLPOUT);  // Sleep out
    __delay_cycles(150000);

    ST7735_sendCommand(ST7735_FRMCTR1); // Frame rate control - normal mode
    ST7735_sendData(0x01);
    ST7735_sendData(0x2C);
    ST7735_sendData(0x2D);

    ST7735_sendCommand(ST7735_FRMCTR2); // Frame rate control - idle mode
    ST7735_sendData(0x01);
    ST7735_sendData(0x2C);
    ST7735_sendData(0x2D);

    ST7735_sendCommand(ST7735_FRMCTR3); // Frame rate control - partial mode
    ST7735_sendData(0x01);
    ST7735_sendData(0x2C);
    ST7735_sendData(0x2D);
    ST7735_sendData(0x01);
    ST7735_sendData(0x2C);
    ST7735_sendData(0x2D);

    ST7735_sendCommand(ST7735_INVCTR);  // Display inversion control
    ST7735_sendData(0x07);

    ST7735_sendCommand(ST7735_PWCTR1);  // Power control
    ST7735_sendData(0xA2);
    ST7735_sendData(0x02);
    ST7735_sendData(0x84);

    ST7735_sendCommand(ST7735_PWCTR2);  // Power control
    ST7735_sendData(0xC5);

    ST7735_sendCommand(ST7735_PWCTR3);  // Power control
    ST7735_sendData(0x0A);
    ST7735_sendData(0x00);

    ST7735_sendCommand(ST7735_PWCTR4);  // Power control
    ST7735_sendData(0x8A);
    ST7735_sendData(0x2A);

    ST7735_sendCommand(ST7735_PWCTR5);  // Power control
    ST7735_sendData(0x8A);
    ST7735_sendData(0xEE);

    ST7735_sendCommand(ST7735_VMCTR1);  // VCOM control
    ST7735_sendData(0x0E);

    ST7735_sendCommand(ST7735_GMCTRP1); // Positive Gamma correction
    ST7735_sendData(0x02);
    ST7735_sendData(0x1C);
    ST7735_sendData(0x07);
    ST7735_sendData(0x12);
    ST7735_sendData(0x37);
    ST7735_sendData(0x32);
    ST7735_sendData(0x29);
    ST7735_sendData(0x2D);
    ST7735_sendData(0x29);
    ST7735_sendData(0x25);
    ST7735_sendData(0x2B);
    ST7735_sendData(0x39);
    ST7735_sendData(0x00);
    ST7735_sendData(0x01);
    ST7735_sendData(0x03);
    ST7735_sendData(0x10);

    ST7735_sendCommand(ST7735_GMCTRN1); // Negative Gamma correction
    ST7735_sendData(0x03);
    ST7735_sendData(0x1D);
    ST7735_sendData(0x07);
    ST7735_sendData(0x06);
    ST7735_sendData(0x2E);
    ST7735_sendData(0x2C);
    ST7735_sendData(0x29);
    ST7735_sendData(0x2D);
    ST7735_sendData(0x2E);
    ST7735_sendData(0x2E);
    ST7735_sendData(0x37);
    ST7735_sendData(0x3F);
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);
    ST7735_sendData(0x02);
    ST7735_sendData(0x10);

    ST7735_sendCommand(ST7735_COLMOD);  // Interface pixel format
    ST7735_sendData(0x05); // 16-bit/pixel RGB 5-6-5

    ST7735_sendCommand(ST7735_MADCTL);  // Memory data access control
    ST7735_sendData(0xC8);

    ST7735_sendCommand(ST7735_CASET);   // Column address set
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);
    ST7735_sendData(0x7F);  // X address: 0 to 127

    ST7735_sendCommand(ST7735_RASET);   // Row address set
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);
    ST7735_sendData(0x00);
    ST7735_sendData(0x9F);  // Y address: 0 to 159

    ST7735_sendCommand(ST7735_NORON);   // Normal display mode on
    __delay_cycles(10000);

    ST7735_sendCommand(ST7735_DISPON);  // Display on
    __delay_cycles(10000);

    P2OUT |= CS_PIN;
}

// Função para enviar comandos ao display
void ST7735_sendCommand(uint8_t command) {
    P2OUT &= ~DC_PIN; // Command mode
    UCB0TXBUF = command;
    while (!(UCB0IFG & UCTXIFG)); // Aguarda transmissão
}

// Função para enviar dados ao display
void ST7735_sendData(uint8_t data) {
    P2OUT |= DC_PIN; // Data mode
    UCB0TXBUF = data;
    while (!(UCB0IFG & UCTXIFG)); // Aguarda transmissão
}



/*
 * Função desenvolvida para escrever caracter na tela
 */

void ST7735_DrawChar(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size) {
  uint8_t line;
  int32_t i, j, col, row;

  // Certifique-se de que o caractere está no intervalo correto
  if (c < '0' || c > '9') return; // Limita aos caracteres de '0' a '9'

  // Percorrer as 5 colunas do caractere
  for (i = 0; i < 5; i++) {
    line = Font[(c - '0') * 5 + i]; // Corrige o índice para os números

    // Para cada linha de bits, processar as 8 linhas verticais
    for (j = 0; j < 8; j++) {
      // Se o bit estiver ligado (caracter é desenhado)
      if (line & 0x1) {
        // Desenhar o pixel com o tamanho ajustado
        for (row = 0; row < size; row++) {
          for (col = 0; col < size; col++) {
            // Desenha o pixel na posição (x + i * size + col, y + j * size + row)
            drawPixel(x + (i * size) + col, y + (j * size) + row, textColor);
          }
        }
      } else {
        // Caso contrário, desenha o pixel de fundo com o tamanho ajustado
        for (row = 0; row < size; row++) {
          for (col = 0; col < size; col++) {
            // Desenha o pixel de fundo
            drawPixel(x + (i * size) + col, y + (j * size) + row, bgColor);
          }
        }
      }
      line >>= 1; // Move para o próximo bit
    }
  }
}

