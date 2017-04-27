// PIC16F1938 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable (All VCAP pin functionality is disabled)
#pragma config PLLEN = ON      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <math.h>
#define _XTAL_FREQ 32000000
#define inputPIN RB5
#define IRLED RB6
#define LCD_E  RB2
#define LCD_RS RB1
#define FireFly1 RA4
#define FireFly2 RA5

void LCD_str(char *);
static void LCD_send4(unsigned char);
static void LCD_cmd(unsigned char);
static void LCD_data(unsigned char);
static void LCD_init();
double NOWTEMP(int);
unsigned int adconvAX(int);
static void pic_init();
static void AXdisp();
void interrupt isr();
void intrInit();
void sendData(unsigned int);
void outHigh();
void outLow();
void stopbit();
void ExecuteInstruction(int);
void FireFly(char);

char SendingMSG[17] = {"Sending Data"};
char SecondMSG[16] = {"made by cutman"};
char testMSG[17]={"Missile arriving"};
char Warning[16]={"Warning!"};
char wakeupMsg[16]={"Wake up!"};
char str[14];

void IRreceive();
int data =0;
int Bconst = 3500;
char SystemID = 1;
unsigned int tmpData = 0;


void main()
{

    pic_init();
    LCD_init();

     for(int r=0;r<10;r++){
          RC3 = 1;
        __delay_ms(100);
         RC3 = 0;
        __delay_ms(100);
     }
     RC3 = 1;

     __delay_ms(100);
     RC3 = 0;
    intrInit();
    FireFly(3);
     while(1){
         AXdisp();
     }

}

void FireFly(char NO){
    FireFly1 = NO&0b01;
    FireFly2 = (NO>>1)&0b01;
}
void interrupt isr(){               //割り込み関数
    GIE = 0;
    if(INTCONbits.INTF==1){
        RC3 = 1;                    //受信確認LEDオン
        IRreceive();                //赤外線データをdataに格納
        ExecuteInstruction(data);   //dataの内容に基づいた動作をする
        data = 0;                   //dataの内容を0にする
        INTCONbits.INTF = 0;     //INIT割り込みフラグを落とす
    }

    RC3 = 0;                        //受信確認LEDオフ
    GIE = 1;
}
void intrInit(){                    //割り込みの初期設定
    INTCONbits.GIE = 1;              //グローバル割り込みを許可
    INTCONbits.PEIE = 1;             //割り込みを許可

    OPTION_REGbits.INTEDG = 0;       //INITピン（RB0）がLowの時に割り込み発生
    INTCONbits.INTE = 1;             //INITピン（RB0）の割り込みを許可

}
void ExecuteInstruction(int data){

    int datatmp = data;
    data = data & 0b00011111;        //上位3bitを消去，命令識別番号・命令内容のみ残す
    if(datatmp>>5 == SystemID){      //dataの内容を右に5bit分シフト．SystemIDのみにする．SystemIDと合致しない場合は実行しない．
        switch(data){
            case 0: //炉壁AC側オン
                break;
            case 1: //炉壁AC側オフ
                break;
            case 2: //炉壁BC側オン
                break;
            case 3: //炉壁BC側オフ
                break;
            case 4: //ハザードブザーオン
                break;
            case 5: //ハザードブザーオフ
                break;
            case 6: //緊急停止命令
                break;
            case 7: //温度取得命令
                RC3 = 1;
                for(int r=0;r<4;r++){
                    __delay_ms(10);
                    tmpData = SystemID ; //systemIDを12bit上位bitにずらし，tmpDataに格納
                    tmpData = tmpData<<2;
                    tmpData = tmpData + r ;     //温度計番号を11bit目に格納
                    tmpData = tmpData<<10;
                    tmpData = tmpData + adconvAX(r); //下位10bitにAD変換結果を代入
                    sendData(tmpData);  //温度情報を赤外線で送信
                    tmpData = 0;

                }
                break;
            default:
                break;
        }
    }
    data = 0;
}
void sendData(unsigned int tmpSend){            //16bitのdataの内容を送信
    unsigned int yj = 0;
    outHigh();              //スタートビット
    for(yj = 1;yj<16385;yj = yj*2){
        if((tmpSend & yj) == yj){
            outHigh();
        }else{
            outLow();
        }
    }
    stopbit();
    IRLED = 0;
}
void outHigh(){         //Highを出力(38kHzの点滅を600us(周期26usを23回繰り返し))
    for(int count = 23;count != 0;count--){
        IRLED = 1;
        __delay_us(13);
        IRLED = 0;
        __delay_us(13);
    }
}
void outLow(){          //Low(LED点灯なし)を出力
    IRLED = 0;
    __delay_us(600);
}
void stopbit(){         //ストップビット送信（識別番号）
    outHigh();
    outLow();
    outHigh();
    outLow();
    outHigh();
}
void IRreceive(){
     int i=0;
    if(inputPIN == 0){          //スタートビットを受信した場合

        __delay_us(400);        //遅延（秒数は実験的に決めた）
        __delay_us(600);
        for(i=1;i<513;i = i*2){     //10bitデータを順次dataに加算（2のn乗は、特定bitだけ1になるため、このようにすれば各bitデータを格納できる）
            if(inputPIN == 0)
                data = data + i;
            __delay_us(600);        //600us遅延
        }

        if(inputPIN == 1)           //ストップビット（識別番号）判定。外れた場合はdataを0にする。
            data = 0;
        __delay_us(600);
        if(inputPIN == 0)
            data = 0;
        __delay_us(600);
        if(inputPIN == 1)
            data = 0;
        __delay_us(600);
        if(inputPIN == 0)
            data = 0;
        __delay_us(600);
        if(inputPIN == 1)
            data = 0;


    }

}
void LCD_str(char *c) {
	unsigned char i,wk;
	for (i=0 ; ; i++) {
		wk = c[i];
		if  (wk == 0x00) {break;}
		LCD_data(wk);
	}

}
// ====================== LCD ??(4bit) ======================
static void LCD_send4(unsigned char c) {
	PORTC = (PORTC & 0x0F) | c;
	LCD_E = 1;
    __delay_us(10);
	LCD_E = 0;

	return;
}

// ==================== LCDにコマンドを送信 =================
static void LCD_cmd(unsigned char cmd) {
	LCD_RS = 0;
	PORTC = (PORTC & 0x0F) | (cmd & 0xF0);
	LCD_E = 1;
	NOP();
	LCD_E = 0;

	PORTC = (PORTC & 0x0F) | ((cmd <<4) & 0xF0);
	LCD_E = 1;
	NOP();
	LCD_E = 0;

    __delay_us(50);
	return;
}

// ==================== LCDにデータを送信=================
static void LCD_data(unsigned char cmd) {
	LCD_RS = 1;
	PORTC = (PORTC & 0x0F) | (cmd & 0xF0);
	LCD_E = 1;
	NOP();
	LCD_E = 0;

	PORTC = (PORTC & 0x0F) | ((cmd <<4) & 0xF0);
	LCD_E = 1;
	NOP();
	LCD_E = 0;

    __delay_us(50);
	return;
}

// ==================== LCD初期化 ===========================
static void LCD_init() {
    __delay_ms(1000);
	LCD_E = 0;
	LCD_RS = 0;

	LCD_send4(0x30);
    __delay_ms(5);
	LCD_send4(0x30);
    __delay_us(100);
	LCD_send4(0x30);
    __delay_ms(1);
	LCD_send4(0x20);
    __delay_ms(1);

	LCD_cmd(0x2C);	// DL=0:4bit ,N=1:2lines ,F=0:5*7dot
	LCD_cmd(0x08);	// Display off
	LCD_cmd(0x0c);	// Display on ,cursor on ,no blink
	LCD_cmd(0x06);	// Entry mode (Inc ,No Shift)
	LCD_cmd(0x01);	// Clear Display

    __delay_ms(2);

	return;
}
double NOWTEMP(int AN){   //現在の温度を返す
    double AXVolt = 0, AXresist = 0,tmp0 = 0;
    long adSum = 0;
    for(int i=0;i<100;i++){                           //AD変換を100回行い，adSumに合計値を入れる
        adSum = adSum +adconvAX(AN);   //AD変換の値を積算
    }
    AXVolt = adSum*0.00488/100;                          //AD変換値の平均値に分解能をかけて，電圧を算出
    AXresist = (4.92-AXVolt)/AXVolt;             //電圧をサーミスタの抵抗値に変換

    tmp0 = 1/(log(AXresist)/Bconst+1/298.15)-273.15;    //サーミスタの温度を計算

    return tmp0;                                 //温度を返す
}

unsigned int adconvAX(int AN)     //サーミスタの電圧を10bitで取得する関数
{
ADCON0 = AN<<2;     //ADCON0のCHS<4:0>の値をANにする（2bit目から始まるため，2bit左詰め）
ADON = 1;	//AD変換ON
__delay_us(20);
GO = 1;	//AD変換待ち
while(GO);	//AD変換終了まで待機
return (ADRESH<<8) + ADRESL;
}

// ==================== PIC初期化 ===========================
static void pic_init() {		// INTOSC 8MHz
    OSCCON     = 0b01110000 ; // 内部クロックは32ＭＨｚとする
    ADCS2 = 0;	//AD変換クロックFosc/32
    ADCS1 = 1;
    ADCS0 = 0;
    ADFM = 1; // AD変換結果は右詰め
    ADNREF = 0; // ネガティブ基準電圧はVss
    ADPREF1 = 0;
    ADPREF0 = 0;//ポジティブ基準電圧はVdd
    ANSELA = 0b00001111; // AN{0:3}をオン
    TRISA = 0b00001111; // RA{0:3}を入力ピンに設定
    TRISB = 0b00100001; // RB0,RB5を入力ピンに設定
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    OPTION_REG = 0b00000000 ; // デジタルI/Oに内部プルアップ抵抗を使用する
    ANSELB     = 0b00000000 ; // AN8-AN13は使用しない全てデジタルI/Oとする
    TRISC      = 0b00000000 ; // ピン(RC)は全て出力に割当てる
    WPUB       = 0b00000001 ; // RB0は内部プルアップ抵抗を指定する
    PORTC      = 0b00000000 ; // RC出力ピンの初期化(全てLOWにする)
	return;
}
static void AXdisp()                       //LCDにtimerの値、温度を表示する関数
{
  double tmpx;
  int tmpx1,tmpx2;
  char tmpstr[14];
  LCD_cmd(0x01);
  __delay_ms(2);

  for(int r=0;r<4;r++){
    tmpx = NOWTEMP(r);                       //tmpxに現在の温度をセット
    tmpx1 = tmpx;
    tmpx2 = (tmpx-tmpx1)*10;

    if(tmpx1 < 0){
        sprintf(tmpstr,"%d%c:NA",SystemID,'A'+r);
    }
    sprintf(tmpstr,"%d%c:%d.%d",SystemID,'A'+r,tmpx1,tmpx2);       //tmpstrに文字データをセット
    LCD_str(tmpstr);                                //LCDにtmpstrを表示

    switch(r){
      case 0:
        LCD_cmd(0b10001000);
        break;
      case 1:
        LCD_cmd(0xC0);
        break;
      case 2:
          LCD_cmd(0b11001000);
        break;
      case 3:
        break;
    }
    __delay_ms(1);
  }
  __delay_ms(1000);
}
