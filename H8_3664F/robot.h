/************************************************************************************
   File名：robot.h                                               (2004/07/06)

   メカトロニクストレーニングロボットのためのＣヘッダー（ライブラリ）プログラム
                               Version 0.3.3

                                                             By  YOU-YOU ROBOTICS
************************************************************************************/
#include <3664F.H>
#include <MACHINE.H>
#include <STDLIB.H>
#include <FLOAT.H>
#include <LIMITS.H>
#include <MATH.H>

/*ロボットの位置と姿勢角の誤差を少なくするには L_L とR_R の値を正確な値にして書き直
                                                                       して下さい。*/
#define L_L 17.4f	/*車輪と車輪との間の距離 [cm]*/
#define R_R 2.9f	/*車輪の半径 [cm]*/

#define THRESHOLD 300 /*障害物を探すために必要な閾値　　　　　　　　　　　　　　　　*/
                      /*センサ信号の読み取り値がこの値より小さいと障害物なしと見なす*/

/***********************************************************************************/

#define KP  55.0f	/*PI速度制御の比例ゲイン（必要なら変更して下さい。）*/
#define KI  3.7f	/*PI速度制御の積分ゲイン（必要なら変更して下さい。）*/

#define T_T 0.0124f    /*制御周期 [sec]*/
#define PI  3.1415926535897932f /*円周率の値*/
#define K_K 3.51409e-2f	/*エンコーダパルス数と車輪回転角度の変換係数*/

/***********************************************************************************
     ギヤボックスの減速比 G=29.8
     ロータリエンコーダの分解能 P=6.0 [pulse/rev]
     ロータリエンコーダのカウント値 = COUNT
      とするとき、
     モータ軸の回転角度[radian]=2*PI*(float)COUNT/P
     車輪の回転角度[radian]=2*PI*(float)COUNT/P/G
     したがって、エンコーダパルス数から車輪回転角度への変換係数 K_K = 2*PI/P/G = 3.51409e-2
******************************************************************************************/

extern long RCOUNT;            /*右側ロータリエンコーダのカウント値*/
extern long LCOUNT;            /*左側ロータリエンコーダのカウント値*/
extern long RCOUNT_1;          /*右側ロータリエンコーダの１時刻前のカウント値*/
extern long LCOUNT_1;          /*左側ロータリエンコーダの１時刻前のカウント値*/
extern long RCOUNT_2;          /*右側ロータリエンコーダの２時刻前のカウント値*/
extern long LCOUNT_2;          /*左側ロータリエンコーダの２時刻前のカウント値*/
extern unsigned int DISTANCE;  /*赤外線測距センサ出力のAD変換値*/
extern unsigned int GNDV;      /*ライントレースセンサのGNDのAD変換値*/
extern unsigned int TRACER;    /*ライントレースセンサ出力のAD変換値*/
extern unsigned int ADVOLT;    /**/
extern float POSITION_X;       /*ロボットのＸ座標値 [cm]*/
extern float POSITION_Y;       /*ロボットのＹ座標値 [cm]*/
extern float ATTITUDE_ANGLE;    /*ロボットの姿勢角 [degree]*/
extern float ATTITUDE_ANGLE0;   /*ロボットの初期姿勢角 [degree]*/
extern float FAI0;             /*ロボットの初期姿勢角 [radian]*/
extern float POSITION_X_1;     /*ロボットの１時刻前のＸ座標値 [cm]*/
extern float POSITION_Y_1;     /*ロボットの１時刻前のＹ座標値 [cm]*/
extern float POSITION_X_2;     /*ロボットの２時刻前のＸ座標値 [cm]*/
extern float POSITION_Y_2;     /*ロボットの２時刻前のＹ座標値 [cm]*/
extern float COSFAI_2;         /**/
extern float SINFAI_2;         /**/
extern float OMEGA_2;          /*ロボットの位置と姿勢角を計算するための変数*/
extern float R_SPEED;          /*右側車輪の移動速度*/
extern float L_SPEED;          /*左側車輪の移動速度*/
extern float SUM_R;            /*右側モータを制御するために使う積算変数*/
extern float SUM_L;            /*左側モータを制御するために使う積算変数*/
extern unsigned char CFLAG;    /*10msecごとにセットされるフラグ*/
extern unsigned char SWDATA;   /*2ビットディップスイッチの値が入っている変数*/
extern unsigned char AUTOFLASH;    /*LEDを自動点滅させるための変数*/
extern unsigned char LINEDETECT_R; /*右側センサがラインを検出したときに１にセットされる変数*/
extern unsigned char LINEDETECT_L; /*左側センサがラインを検出したときに１にセットされる変数*/
extern unsigned char LINESENSE;    /*ライントレース用のセンサを働かせるためのフラグ*/
extern char *_D_ROM, *_B_BGN, *_B_END, *_D_BGN, *_D_END;
extern void _INITSCT(void);



void _INITSCT(void)  /*データセクションを初期化するプログラム*/
{
  char *p, *q ;

  /* 未初期化データ領域をゼロで初期化 */
  for (p=_B_BGN; p<_B_END ; p++)
    *p=0;

  /*初期化データをROM 上からRAM 上へコピー */
  for (p=_D_BGN, q=_D_ROM; p<_D_END; p++, q++)
    *p=*q;
}

float sfabs(float x)  /*  float 型数値の絶対値をとる関数  */
{
  if(x<0.0f) x=-x;
  return x;
}

float sin0(float r)  /*  0～PI/2までのsin関数  */
{
  float sn,r2;
  if(r<2.6179938779914943e-1f)/*0<=r<PI/12の場合*/
  {
    r2=r*r;
    sn=2.7557319223985890e-6f;
    sn*=r2;
    sn-=1.9841269841269841e-4f;
    sn*=r2;
    sn+=8.3333333333333333e-3f;
    sn*=r2;
    sn-=1.6666666666666666e-1f;
    sn*=r2;
    sn+=1.0000000000000000f;
    sn*=r;
 }
  else if(r<5.2359877559829887e-1f)/*PI/12<=r<PI/6の場合*/
  {
    r-=2.6179938779914943e-1f;
    sn=2.6618326341740197e-6f;
    sn*=r;
    sn+=6.4191231424236300e-6f;
    sn*=r;
    sn-=1.9165194966052942e-4f;
    sn*=r;
    sn-=3.5947089597572328e-4f;
    sn*=r;
    sn+=8.0493818857422357e-3f;
    sn*=r;
    sn+=1.0784126879271698e-2f;
    sn*=r;
    sn-=1.6098763771484471e-1f;
    sn*=r;
    sn-=1.2940952255126038e-1f;
    sn*=r;
    sn+=9.6592582628906828e-1f;
    sn*=r;
    sn+=2.5881904510252076e-1f;
  }
  else if(r<7.8539816339744830e-1f)/*PI/6<=r<PI/4の場合*/
  {
    r-=5.2359877559829887e-1f;
    sn=2.3865338508169054e-6f;
    sn*=r;
    sn+=1.2400793650793650e-5f;
    sn*=r;
    sn-=1.7183043725881719e-4f;
    sn*=r;
    sn-=6.9444444444444444e-4f;
    sn*=r;
    sn+=7.2168783648703220e-3f;
    sn*=r;
    sn+=2.0833333333333333e-2f;
    sn*=r;
    sn-=1.4433756729740644e-1f;
    sn*=r;
    sn-=2.5000000000000000e-1f;
    sn*=r;
    sn+=8.6602540378443864e-1f;
    sn*=r;
    sn+=5.0000000000000000e-1f;
  }
  else if(r<1.0471975511965977f)/*PI/4<=r<PI/3の場合*/
  {
    r-=7.8539816339744830e-1f;
    sn=1.9485967294602830e-6f;
    sn*=r;
    sn+=1.7537370565142547e-5f;
    sn*=r;
    sn-=1.4029896452114038e-4f;
    sn*=r;
    sn-=9.8209275164798267e-4f;
    sn*=r;
    sn+=5.8925565098878960e-3f;
    sn*=r;
    sn+=2.9462782549439480e-2f;
    sn*=r;
    sn-=1.1785113019775792e-1f;
    sn*=r;
    sn-=3.5355339059327376e-1f;
    sn*=r;
    sn+=7.0710678118654752e-1f;
    sn*=r;
    sn+=7.0710678118654752e-1f;
  }
  else if(r<1.3089969389957471f)/*PI/3<=r<5PI/12の場合*/
  {
    r-=1.0471975511965977f;
    sn=1.3778659611992945e-6f;
    sn*=r;
    sn+=2.1478804657352148e-5f;
    sn*=r;
    sn-=9.9206349206349206e-5f;
    sn*=r;
    sn-=1.2028130608117203e-3f;
    sn*=r;
    sn+=4.1666666666666666e-3f;
    sn*=r;
    sn+=3.6084391824351610e-2f;
    sn*=r;
    sn-=8.3333333333333333e-2f;
    sn*=r;
    sn-=4.3301270189221932e-1f;
    sn*=r;
    sn+=5.0000000000000000e-1f;
    sn*=r;
    sn+=8.6602540378443864e-1f;
  }
  else /*5PI/12<=r<PI/2*/
  {
    r-=1.3089969389957471f;
    sn=7.1323590471373666e-7f;
    sn*=r;
    sn+=2.3956493707566177e-5f;
    sn*=r;
    sn-=5.1352985139389040e-5f;
    sn*=r;
    sn-=1.3415636476237059e-3f;
    sn*=r;
    sn+=2.1568253758543396e-3f;
    sn*=r;
    sn+=4.0246909428711178e-2f;
    sn*=r;
    sn-=4.3136507517086793e-2f;
    sn*=r;
    sn-=4.8296291314453414e-1f;
    sn*=r;
    sn+=2.5881904510252076e-1f;
    sn*=r;
    sn+=9.6592582628906828e-1f;
  }
  return sn;
}

float cos0(float r)  /*  0～PI/2までのcos関数  */
{
  float cs,r2;
  if(r<2.6179938779914943e-1f)/*0<=r<PI/12の場合*/
  {
    r2=r*r;
    cs=2.4801587301587301e-5f;
    cs*=r2;
    cs-=1.3888888888888888e-3f;
    cs*=r2;
    cs+=4.1666666666666666e-2f;
    cs*=r2;
    cs-=5.0000000000000000e-1f;
    cs*=r2;
    cs+=1.0000000000000000f;
  }
  else if(r<5.2359877559829887e-1f)/*PI/12<=r<PI/6の場合*/
  {
    r-=2.6179938779914943e-1f;
    cs=-7.1323590471373666e-7f;
    cs*=r;
    cs+=2.3956493707566177e-5f;
    cs*=r;
    cs+=5.1352985139389040e-5f;
    cs*=r;
    cs-=1.3415636476237059e-3f;
    cs*=r;
    cs-=2.1568253758543396e-3f;
    cs*=r;
    cs+=4.0246909428711178e-2f;
    cs*=r;
    cs+=4.3136507517086793e-2f;
    cs*=r;
    cs-=4.8296291314453414e-1f;
    cs*=r;
    cs-=2.5881904510252076e-1f;
    cs*=r;
    cs+=9.6592582628906828e-1f;
  }
  else if(r<7.8539816339744830e-1f)/*PI/6<=r<PI/4の場合*/
  {
    r-=5.2359877559829887e-1f;
    cs=-1.3778659611992945e-6f;
    cs*=r;
    cs+=2.1478804657352148e-5f;
    cs*=r;
    cs+=9.9206349206349206e-5f;
    cs*=r;
    cs-=1.2028130608117203e-3f;
    cs*=r;
    cs-=4.1666666666666666e-3f;
    cs*=r;
    cs+=3.6084391824351610e-2f;
    cs*=r;
    cs+=8.3333333333333333e-2f;
    cs*=r;
    cs-=4.3301270189221932e-1f;
    cs*=r;
    cs-=5.0000000000000000e-1f;
    cs*=r;
    cs+=8.6602540378443864e-1f;
  }
  else if(r<1.0471975511965977f)/*PI/4<=r<PI/3の場合*/
  {
    r-=7.8539816339744830e-1f;
    cs=-1.9485967294602830e-6f;
    cs*=r;
    cs+=1.7537370565142547e-5f;
    cs*=r;
    cs+=1.4029896452114038e-4f;
    cs*=r;
    cs-=9.8209275164798267e-4f;
    cs*=r;
    cs-=5.8925565098878960e-3f;
    cs*=r;
    cs+=2.9462782549439480e-2f;
    cs*=r;
    cs+=1.1785113019775792e-1f;
    cs*=r;
    cs-=3.5355339059327376e-1f;
    cs*=r;
    cs-=7.0710678118654752e-1f;
    cs*=r;
    cs+=7.0710678118654752e-1f;
  }
  else if(r<1.3089969389957471f)/*PI/3<=r<5PI/12の場合*/
  {
    r-=1.0471975511965977f;
    cs=-2.3865338508169054e-6f;
    cs*=r;
    cs+=1.2400793650793650e-5f;
    cs*=r;
    cs+=1.7183043725881719e-4f;
    cs*=r;
    cs-=6.9444444444444444e-4f;
    cs*=r;
    cs-=7.2168783648703220e-3f;
    cs*=r;
    cs+=2.0833333333333333e-2f;
    cs*=r;
    cs+=1.4433756729740644e-1f;
    cs*=r;
    cs-=2.5000000000000000e-1f;
    cs*=r;
    cs-=8.6602540378443864e-1f;
    cs*=r;
    cs+=5.0000000000000000e-1f;
  }
  else /*5PI/12<=r<PI/2*/
  {
    r-=1.3089969389957471f;
    cs=-2.6618326341740197e-6f;
    cs*=r;
    cs+=6.4191231424236300e-6f;
    cs*=r;
    cs+=1.9165194966052942e-4f;
    cs*=r;
    cs-=3.5947089597572328e-4f;
    cs*=r;
    cs-=8.0493818857422357e-3f;
    cs*=r;
    cs+=1.0784126879271698e-2f;
    cs*=r;
    cs+=1.6098763771484471e-1f;
    cs*=r;
    cs-=1.2940952255126038e-1f;
    cs*=r;
    cs-=9.6592582628906828e-1f;
    cs*=r;
    cs+=2.5881904510252076e-1f;
  }
  return cs;
}

float sin(float x)  /*  大域的sin関数  */
{
  char sign;
  float y;
  sign=0;
  
  if(x<0.0f)   /*xが負なら正に変換、負号フラグセット*/
  {
    x=-x;
    sign=1;
  }
   while(x>=6.2831853071795864f)  /* 2*PIで割った余りを算出*/
  {
    x-=6.2831853071795864f;
  }
  if(x<1.5707963267948966f)       /* 0<= x <PI/2の場合*/
  {
    y=sin0(x);
  }
  else if(x<PI)                  /* PI/2<= x <PI の場合*/
  {
    x-=1.5707963267948966f;      /* x=x-PI/2 */
    y=cos0(x);
  }
  else if(x<4.7123889803846898f) /* PI<= x <3*PI/2 の場合*/
  {
    x-=PI;                       /* x=x-PI */
    y=-sin0(x);
  }
  else                           /* 3*PI/2<= x <2*PI の場合*/
  {
    x-=4.7123889803846898f;      /* x=x-3*PI/2 */
    y=-cos0(x);
  }
  if(sign==1) y=-y;
  return y;
}

float cos(float x)  /*  大域的cos関数  */
{
  float y;
  if(x<0.0f) x=-x;
  while(x>=6.2831853071795864f)  /* 2*PIで割った余りを算出*/
  {
    x-=6.2831853071795864f;
  }
  if(x<1.5707963267948966f)      /* 0<= x <PI/2の場合*/
  {
    y=cos0(x);
  }
  else if(x<PI)                  /* PI/2<= x <PI の場合*/
  {
    x-=1.5707963267948966f;      /* x=x-PI/2 */
    y=-sin0(x);
  }
  else if(x<4.7123889803846898f) /* PI<= x <3*PI/2 の場合*/
  {
    x-=PI;                       /* x=x-PI */
    y=-cos0(x);
  }
  else                           /* 3*PI/2<= x <2*PI の場合*/
  {
    x-=4.7123889803846898f;      /* x=x-3*PI/2 */
    y=sin0(x);
  }
  return y;
}

float sqrt(float x)  /*  スクウェアルート  */
{
  float new_x, old_x;
  if(x>1.0f)
  {
    new_x=x;
    while(1)
    {
      old_x=new_x;
      new_x=0.5f*(old_x+x/old_x);
      if(sfabs((new_x-old_x)/old_x)<0.00001f) break;
    }
  }
  else if(x>0.000001f)   /* 0.000001<= x <1 の場合は逆数のルートを求める。*/
  {
    x=1.0f/x;
    new_x=x;
    while(1)
    {
      old_x=new_x;
      new_x=0.5f*(old_x+x/old_x);
      if(sfabs((new_x-old_x)/old_x)<0.00001f) break;
    }
    new_x=1.0f/new_x;        /*逆数で求めたので元に戻して答えとする。*/
  }
  else                       /* x<0.000001 の場合はゼロとみなす。*/
  {
    new_x=0.0f;
  }
  return new_x;
}

/***********************************************************************************
  位置と姿勢角の計算公式は以下のURLを参照下さい
       http://kamuy.elec.muroran-it.ac.jp/study/kinematics/estimation.html
************************************************************************************/
void current_status(void) /*の位置と姿勢角をバックグラウンドで計算するＣプログラムルーチン*/
{
  float cosfai_1,sinfai_1,fai_1,romega_1,lomega_1,omega_1,theta_r,theta_l;
  long count_r,count_l;
  count_r=RCOUNT;    /*処理中にRCOUNTの値が書き変わる可能性があるのでラッチ*/
  count_l=LCOUNT;    /*処理中にLCOUNTの値が書き変わる可能性があるのでラッチ*/
  theta_r=K_K*(float)count_r;
  theta_l=K_K*(float)count_l;
/*---------------車輪の移動速度の計算-------------*/
  R_SPEED=K_K*(float)RCOUNT_1;
  L_SPEED=K_K*(float)LCOUNT_1;
  R_SPEED=R_R*(theta_r-R_SPEED)/T_T;
  L_SPEED=R_R*(theta_l-L_SPEED)/T_T;
/*--------------ここまでが移動速度の計算----------*/
  romega_1=K_K*(float)(count_r-RCOUNT_2);
  lomega_1=K_K*(float)(count_l-LCOUNT_2);
  omega_1=0.5f*(romega_1+lomega_1)/T_T;
  fai_1=K_K*(float)(RCOUNT_1-LCOUNT_1);
  fai_1=R_R*fai_1/L_L;
  cosfai_1=cos(fai_1+FAI0);
  sinfai_1=sin(fai_1+FAI0);
  POSITION_X_1=0.25f*R_R*T_T*(omega_1*cosfai_1+OMEGA_2*COSFAI_2);
  POSITION_X_1+=POSITION_X_2;
  POSITION_Y_1=0.25f*R_R*T_T*(omega_1*sinfai_1+OMEGA_2*SINFAI_2);
  POSITION_Y_1+=POSITION_Y_2;
  POSITION_X=2.0f*POSITION_X_1-POSITION_X_2;                /*現時点のＸ座標値を推定*/
  POSITION_Y=2.0f*POSITION_Y_1-POSITION_Y_2;                /*現時点のＹ座標値を推定*/
  ATTITUDE_ANGLE=0.572958e+2f*R_R*(theta_r-theta_l)/L_L+ATTITUDE_ANGLE0;/*現時点の姿勢角*/
  RCOUNT_2=RCOUNT_1;
  LCOUNT_2=LCOUNT_1;
  RCOUNT_1=count_r;
  LCOUNT_1=count_l;
  OMEGA_2=omega_1;
  POSITION_X_2=POSITION_X_1;
  POSITION_Y_2=POSITION_Y_1;
  COSFAI_2=cosfai_1;
  SINFAI_2=sinfai_1;
}

void wait(float sec) /* 指定した時間だけ待つ関数。 引数secの単位は秒*/
{
  unsigned int i;
  i=(unsigned int)(sec/T_T);
  while(i)
  {
    if(CFLAG)
    {
      CFLAG=0;
      i--;
    }
  }
}

unsigned char input0(void)  /*ユーザー開放入力端子 P76 の電圧レベルを取り込む関数*/
{
  return (unsigned char)IO.PDR7.BIT.B6;
}

unsigned char input1(void)  /*ユーザー開放入力端子 P87 の電圧レベルを取り込む関数*/
{
  return (unsigned char)IO.PDR8.BIT.B7;
}

void output0(unsigned char revel)  /*ユーザー開放出力端子 P85 に電圧を出力する関数*/
{
  if(revel==1) IO.PDR8.BIT.B5=1;
  else IO.PDR8.BIT.B5=0;
}

void output1(unsigned char revel)  /*ユーザー開放出力端子 P86 に電圧を出力する関数*/
{
  if(revel==1) IO.PDR8.BIT.B6=1;
  else IO.PDR8.BIT.B6=0;
}

void pwm(unsigned int duty)  /*ユーザー開放PWM出力端子 P84 にPWM信号を出力する関数*/
{
  if(duty>799) duty=799;
  TW.GRD=duty;
}

void linesensor_on(char on)  /*ラインセンサの電源をオンにする関数*/
{
	if(on!=0) on=1;               /*0以外の値が設定されたときには1とみなす*/
	if(on==1) IO.PDR7.BIT.B4=0;   /*引数１で電源オン*/
	else IO.PDR7.BIT.B4=1;        /*引数０で電源オフ*/
}

void rangesensor_on(char on) /*赤外線測距センサの電源をオンにする関数*/
{
	if(on!=0) on=1;               /*0以外の値が設定されたときには1とみなす*/
	if(on==1) IO.PDR7.BIT.B5=0;   /*引数１で電源オン*/
	else IO.PDR7.BIT.B5=1;        /*引数０で電源オフ*/
}

void stop(void)  /*ロボットを停止させる関数*/
{
  IO.PDR5.BIT.B4=0;  /*右側車輪回転ストップ*/
  IO.PDR5.BIT.B5=0;
  IO.PDR5.BIT.B6=0;  /*左側車輪回転ストップ*/
  IO.PDR5.BIT.B7=0;
  TW.GRB=1;
  TW.GRC=1;
  SUM_R=0.0f;        /*右側積分器クリア*/
  SUM_L=0.0f;        /*左側積分器クリア*/
}

/*************************************************************************************
　デューティを与えて制御なしでロボットを動かす関数
　正のデューティ（0～799)を与えること
**************************************************************************************/
void move_forward(unsigned int lduty, unsigned int rduty) /*前進する関数*/
{
    
    if(lduty>799) lduty=799;
    if(rduty>799) rduty=799;
    TW.GRC=lduty;/*左側デューティの変更*/
    TW.GRB=rduty;/*右側デューティの変更*/
    IO.PDR5.BIT.B6=1;      /*左車輪の回転方向が正転になるように*/
    IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
    IO.PDR5.BIT.B4=1;      /*右車輪の回転方向が正転になるように*/
    IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
}

void move_backward(unsigned int lduty, unsigned int rduty) /*後退する関数*/
{
    if(lduty>799) lduty=799;
    if(rduty>799) rduty=799;
    TW.GRC=lduty; /*左側デューティの変更*/
    TW.GRB=rduty; /*右側デューティの変更*/
    IO.PDR5.BIT.B6=0;      /*左車輪の回転方向が逆転になるように*/
    IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
    IO.PDR5.BIT.B4=0;      /*右車輪の回転方向が逆転になるように*/
    IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
}
	
/***************************************************************************************
  右車輪と左車輪の回転速度[cm/sec]を与えて指定した距離[cm]だけロボットを移動させる関数
  その場旋回させるような車輪の回転速度の与え方は禁止。移動距離distanceは２つの車輪の中
  央位置の移動距離で、正の値で与えます。lspeed,rspeedは正でも負でもＯＫ。
****************************************************************************************/
void move_s(float lspeed, float rspeed, float distance)
{
  float error_r,error_l,u_r,u_l,x0,y0,s,dx,dy;
  int fg;
  s=0.0f;
  x0=POSITION_X_1;
  y0=POSITION_Y_1;
  fg=1;
  while(s<distance)
  {
    if(CFLAG)
    {
      CFLAG=0;
      error_r=rspeed-R_SPEED;  /*右側制御偏差の計算*/
      error_l=lspeed-L_SPEED;  /*左側制御偏差の計算*/
      SUM_R+=error_r;          /*右側の積分の計算*/
      SUM_L+=error_l;          /*左側の積分の計算*/
      u_r=KP*error_r+KI*SUM_R; /*右側のPI制御量*/
      u_l=KP*error_l+KI*SUM_L; /*左側のPI制御量*/

      if(fg>0)
      {
        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/

        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/
      }
      else
      {
        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/

        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/
      }
      fg*=-1;

      dx=POSITION_X_1-x0;
      dy=POSITION_Y_1-y0;
      s+=sqrt(dx*dx+dy*dy);
      x0=POSITION_X_1;
      y0=POSITION_Y_1;
    }
  }
  stop();
}

/***************************************************************************************
  右車輪と左車輪の回転速度[cm/sec]を与えて指定した時間[sec]だけロボットを移動させる関数
  その場旋回させるような車輪の回転速度の与え方もＯＫです。
****************************************************************************************/
void move_t(float lspeed, float rspeed, float sec)
{
  float error_r,error_l,u_r,u_l;
  unsigned int i;
  int fg;
  i=(unsigned int)(sec/T_T);
  fg=1;
  while(i)
  {
    if(CFLAG)                  /*10.24msec毎にCFLAGが１にセットされるので*/
    {                          /*以降の処理が10.24msec毎に繰り返される*/
      CFLAG=0;
      i--;
      error_r=rspeed-R_SPEED;  /*右側制御偏差の計算*/
      error_l=lspeed-L_SPEED;  /*左側制御偏差の計算*/
      SUM_R+=error_r;          /*右側の積分の計算*/
      SUM_L+=error_l;          /*左側の積分の計算*/
      u_r=KP*error_r+KI*SUM_R; /*右側のPI制御量*/
      u_l=KP*error_l+KI*SUM_L; /*左側のPI制御量*/

      if(fg>0)
      {
        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/

        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/
      }
      else
      {
        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/

        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/
      }
      fg*=-1;
    }
  }
  stop();
}

/**************************************************************************************
  車体の回転速度[deg/sec]を与えて指定した角度[deg]だけロボットをその場旋回させる関数
  引数のspeedには正の値を与え、angleに正負の値を与えて回転方向を指示します。正なら反時
  計まわり、負なら時計まわりです。
***************************************************************************************/
void rotate(float speed, float angle)
{
  float rspeed,lspeed,error_r, error_l,u_r,u_l,angle0;
  int fg;
  angle0=ATTITUDE_ANGLE;
  speed=sfabs(speed);         /*speedは正の値で与えることになっているので念のため*/
  if(angle>0.0f)              /*angleが正なら反時計まわりの旋回*/
  {
    rspeed=8.7266462599716478e-3f*speed*L_L;  /*右車輪に正回転目標値を設定*/
    lspeed=-rspeed;                          /*左車輪に逆回転目標値を設定*/
  }
  else                        /*angleが負なら時計まわりの旋回*/
  {
    lspeed=8.7266462599716478e-3f*speed*L_L;    /*左車輪に正回転目標値を設定*/
    rspeed=-lspeed;           /*右車輪に逆回転目標値を設定*/
  }
  angle=sfabs(angle);         /*正の値同士で旋回角度を比較したいため正値に変換*/
  fg=1;
  while(sfabs(ATTITUDE_ANGLE-angle0)<angle)
  {
    if(CFLAG)
    {
      CFLAG=0;
      error_r=rspeed-R_SPEED; /*右側制御偏差の計算*/
      error_l=lspeed-L_SPEED; /*左側制御偏差の計算*/
      SUM_R+=error_r;         /*右側の積分の計算*/
      SUM_L+=error_l;         /*左側の積分の計算*/
      u_r=KP*error_r+KI*SUM_R;/*右側のPI制御*/
      u_l=KP*error_l+KI*SUM_L;/*左側のPI制御*/

      if(fg>0)
      {
        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/

        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/
      }
      else
      {
        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/

        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/
      }
      fg*=-1;
    }
  }
  stop();
}

/***********************************************************************************************
  車体の移動速度[cm/sec]を与えて指定した時間[sec]だけ黒色ラインをトレースする関数
  引数のspeedには正の値を与えます。sec は0.01sec単位で設定可能。
  ライン検出センサが黒色ラインを検出すると指定した時間が経過していなくても関数から抜け出ます。
************************************************************************************************/
unsigned char linetrace(float speed, float sec)
{
  unsigned int i;
  int fg;
  float adjust, error_r, error_l,u_r,u_l,deviation,shift,kp,ki;
  unsigned char flash, result;
/******ラインのトラッキング感度を調節するときは次行のkpの値を変えて下さい***/
  kp=0.05f;          /* ラインからのずれをなくすための比例ゲイン*/
/***************************************************************************/
  ki=0.0001f;        /* ラインからのずれをなくすための積分ゲイン*/
  shift=0.0f;        /* ラインからのずれを反映する積算変数*/
  LINESENSE=1;       /* ライントレースセンサ・イネーブル*/
  wait(0.1f);        /* センサ回路のウォームアップ待ち*/
  LINEDETECT_R=0;    /* 右側ライン検出フラグのクリア*/
  LINEDETECT_L=0;    /* 左側ライン検出フラグのクリア*/
  flash=AUTOFLASH;   /*AUTOFLASHの値を一時的にセーブ*/
  AUTOFLASH=5;       /*表示用LEDをプログラムから制御できるように設定*/
  i=(unsigned int)(sec/0.01f);
  fg=1;
  while(i)          /*指定した時間が経過するとwhileループを抜けます。*/
  {
    if(CFLAG)
    {
      CFLAG=0;
      i--;
      deviation=(float)TRACER-(float)GNDV;
      if(deviation>=0)   /*ラインからのずれに応じて表示用LEDを点灯*/
      {
        IO.PDR5.BIT.B2=1;   /*ずれが正なら右側LEDを点灯*/
        IO.PDR5.BIT.B3=0;
      }
      else
      {
        IO.PDR5.BIT.B2=0;   /*ずれが非正なら左側LEDを点灯*/
        IO.PDR5.BIT.B3=1;
      }
      shift+=deviation;
      adjust=kp*deviation+ki*shift; /*ライントラッキングの修正量*/
      error_r=speed-adjust-R_SPEED; /*右側制御偏差の計算*/
      error_l=speed+adjust-L_SPEED; /*左側制御偏差の計算*/
      SUM_R+=error_r;               /*右側の積分の計算*/
      SUM_L+=error_l;               /*左側の積分の計算*/
      u_r=KP*error_r+KI*SUM_R;      /*右側のPI制御*/
      u_l=KP*error_l+KI*SUM_L;      /*左側のPI制御*/

      if(fg>0)
      {
        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/

        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/
      }
      else
      {
        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/

        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/
      }
      fg*=-1;
    }
    if((LINEDETECT_R==1)||(LINEDETECT_L==1)) break; /*ラインを検出したらwhileループを抜ける*/
  }
  stop();
  if(LINEDETECT_R==1)
  {
    result=1;
    IO.PDR5.BIT.B2=1;   /*右側LEDを点灯*/
    IO.PDR5.BIT.B3=0;
  }
  else if(LINEDETECT_L==1)
  {
    result=2;
    IO.PDR5.BIT.B2=0;   /*左側LEDを点灯*/
    IO.PDR5.BIT.B3=1;
  }
  else
  {
    result=0;           /*指定した時間が経過して終了した場合*/
    AUTOFLASH=flash;    /* AUTOFLASHの値を復元*/
  }
  LINESENSE=0;          /*ライントレースセンサをディスエーブル*/
  return result;
}


/********************************************************************************************
  その場旋回しながらポールや風船などの障害物を発見する関数
  引数speed[deg/sec]は旋回スピードで正の値を与え、angleには正負の値を与えて回転方向を指示し
  ます。正なら反時計まわり、負なら時計まわりです。障害物を検出すると、指定した角度まで旋回し
  なくても障害物をほぼ正面にみて停止し、関数から抜けます。関数の戻り値は障害物までの距離[cm]。
  測定誤差を考慮するため、連続した観測値５点の移動平均値から最短距離の向きと距離を求めている。
  ただし、距離が遠い場合はバックグラウンドノイズが無視できないので、DISTANCEの値がTHRESHOLD
  よりも小さいときはノイズを測っていると見なす。
  障害物はあるが、最短距離を見つけ出す前に探索を終了した場合には最も近い距離を負の値で返す。
  障害物がない場合は100.0fを返す。
*********************************************************************************************/
float find_obstacle(float speed, float angle)
{
  float rspeed,lspeed,error_r, error_l,u_r,u_l,angle0,ang[5],av[3],d,v;
  unsigned int i,dis[5];
  int fg;
  angle0=ATTITUDE_ANGLE;      /*現在のロボットの向きをangle0に保存*/
  speed=sfabs(speed);         /*speedは正の値で与えることになっているので念のため*/
  if(angle>0.0f)              /*angleが正なら反時計まわりの旋回*/
  {
    rspeed=8.7266462599716478e-3f*speed*L_L;  /*右車輪に正回転目標値を設定*/
    lspeed=-rspeed;                          /*左車輪に逆回転目標値を設定*/
  }
  else                        /*angleが負なら時計まわりの旋回*/
  {
    lspeed=8.7266462599716478e-3f*speed*L_L;    /*左車輪に正回転目標値を設定*/
    rspeed=-lspeed;           /*右車輪に逆回転目標値を設定*/
  }
  angle=sfabs(angle);         /*正の値同士で旋回角度を比較したいため正値に変換*/
  for(i=0;i<5;i++)            /*角度配列を0.0に初期化*/
  {
    dis[i]=THRESHOLD;
    ang[i]=0.0f;
  }
  av[0]=(float)THRESHOLD;     /*バックグラウンドノイズ値で初期化*/
  av[1]=(float)THRESHOLD;     /*バックグラウンドノイズ値で初期化*/
  d=1.0f;                    /*dはcm単位に換算した距離値を表すが、とりあえずは1としておく*/
  fg=1;                      /*fgは左右の車輪の速さを交互に制御するためのフラグ*/
 /*指定した旋回スピードで旋回開始。ただし、指定された角度だけ旋回するとループを抜ける。*/
  while(sfabs(ATTITUDE_ANGLE-angle0)<angle)
  {
    if(CFLAG)
    {
      CFLAG=0;
      error_r=rspeed-R_SPEED; /*右側制御偏差の計算*/
      error_l=lspeed-L_SPEED; /*左側制御偏差の計算*/
      SUM_R+=error_r;         /*右側の積分の計算*/
      SUM_L+=error_l;         /*左側の積分の計算*/
      u_r=KP*error_r+KI*SUM_R;/*右側のPI制御*/
      u_l=KP*error_l+KI*SUM_L;/*左側のPI制御*/

      if(fg>0)
      {
        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/

        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/
      }
      else
      {
        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/

        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/
      }
      fg*=-1;
      /*先頭に新しい観測距離データを付け加えて新たなデータセットを作成*/
      dis[4]=dis[3];
      dis[3]=dis[2];
      dis[2]=dis[1];
      dis[1]=dis[0];
      dis[0]=DISTANCE;
      if(dis[0]<=THRESHOLD) dis[0]=THRESHOLD; /*ノイズに埋もれる距離はTHRESHOLDとおく*/
     /*距離観測を行ったときのロボットの向きも対応させて記憶しておく*/
      ang[4]=ang[3];
      ang[3]=ang[2];
      ang[2]=ang[1];
      ang[1]=ang[0];
      ang[0]=ATTITUDE_ANGLE;
     /*移動平均の計算：av[0]=(dis[0]+dis[1]+dis[2])/3, av[1]=(dis[1]+dis[2]+dis[3])/3, */
     /*            av[2]=(dis[2]+dis[3]+dis[4])/3  ただし、av[2],av[1]は過去に計算済み */
      av[2]=av[1];
      av[1]=av[0];
      av[0]=(float)dis[0];
      av[0]+=(float)dis[1];
      av[0]+=(float)dis[2];
      av[0]*=3.3333333333333333e-1f;
      /*最短距離（極大値）を過ぎたかどうかの判定*/
      if(av[0]-av[1]<0.0f)
      {
        d=-1.0f;        /*最短距離（極大値）を見つけたことを示す印*/
        v=4.8875855327468230e-3f*(float)dis[1];/*最短距離のA/D変換値を電圧値に換算*/
      }
    }
    if(d<0.0f) break;  /*最短距離（極大値）を見つけていたなら探索ループを抜ける*/
  }
  stop();
  if(sfabs(ATTITUDE_ANGLE-angle0)-angle >= 0.0f) /*最短距離（極大値）を*/
  {                                              /*見つけれなかった場合*/
    if((dis[0]==THRESHOLD)&&(dis[3]==THRESHOLD))  /*障害物が無かった場合*/
    {
      d=100.0f;            /*そのときは距離を100cmとする*/
    }
    else /*極大値にまだ至らなかった場合*/
    {
      v=4.8875855327468230e-3f*(float)dis[0];
      d=0.5508f;
      d*=v;
      d-=8.5941f;
      d*=v;
      d+=50.26f;
      d*=v;
      d-=136.51f;
      d*=v;
      d+=166.2f; /*最短距離の計算完了*/
      d=-d;      /*極大値ではないので負の距離を出力*/
    }
  }                                                      
  else
  {
/* 出力電圧vと距離dとの関係を実測すると、概ね次の式で近似できる。*/
/*  d=0.5508*v^4-8.5941*v^3+50.26*v^2-136.51*v+166.2 　　　　　　*/
/* この式を使ってセンサから障害物までの距離を計算する。　　　　　*/
  d=0.5508f;
  d*=v;
  d-=8.5941f;
  d*=v;
  d+=50.26f;
  d*=v;
  d-=136.51f;
  d*=v;
  d+=166.2f; /*最短距離の計算完了。単位はcm*/
  }
  return d;
}

/********************************************************************************************
  黒色ラインを発見する関数 
  lspeed, rspeed には左車輪と右車輪の回転速度[cm/sec]を与えます。secの単位は秒で、0.01sec単位
  で設定可能。黒色ラインを見つけると、指定した時間が経過しなくても停止してこの関数から抜けます。
  リターン値は右側ライン検出センサが黒色ラインを見つけた場合は"-1"、左側ライン検出センサが見つ
  けた場合は"1"です。指定時間内にラインを検出しなかった場合は"0"を出力します。
*********************************************************************************************/
unsigned char find_line(float lspeed, float rspeed, float sec)
{
  float error_r,error_l,u_r,u_l;
  unsigned int i;
  int fg;
  unsigned char result;
  LINEDETECT_R=0;                /*右側ライン検出フラグのクリア*/
  LINEDETECT_L=0;                /*左側ライン検出フラグのクリア*/
  i=(unsigned int)(sec/T_T);     /*指定時間に見合った繰り返し回数の計算*/
  fg=1;
  while(i)
  {
    if(CFLAG)
    {
      CFLAG=0;
      i--;
      error_r=rspeed-R_SPEED;  /*右側制御偏差の計算*/
      error_l=lspeed-L_SPEED;  /*左側制御偏差の計算*/
      SUM_R+=error_r;          /*右側の積分の計算*/
      SUM_L+=error_l;          /*左側の積分の計算*/
      u_r=KP*error_r+KI*SUM_R; /*右側のPI制御量*/
      u_l=KP*error_l+KI*SUM_L; /*左側のPI制御量*/

      if(fg>0)
      {
        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/

        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/
      }
      else
      {
        /*左側モータへの操作量の出力*/
        if(u_l>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B6=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B7=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_l=-u_l;
          IO.PDR5.BIT.B6=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B7=1;      /*ポートに出力を出す              */
        }
        if(u_l>=799.0f) u_l=799.0f;
        TW.GRC=(unsigned int)u_l;/*デューティの変更*/

        /*右側モータへの操作量の出力*/
        if(u_r>=0.0f)            /*前進指令の場合*/
        {
          IO.PDR5.BIT.B4=1;      /*車輪の回転方向が正転になるように*/
          IO.PDR5.BIT.B5=0;      /*ポートに出力を出す              */
        }
        else                     /*後退指令の場合*/
        {
          u_r=-u_r;
          IO.PDR5.BIT.B4=0;      /*車輪の回転方向が逆転になるように*/
          IO.PDR5.BIT.B5=1;      /*ポートに出力を出す              */
        }
        if(u_r>=799.0f) u_r=799.0f;
        TW.GRB=(unsigned int)u_r;/*デューティの変更*/
      }
      fg*=-1;
    }
    if((LINEDETECT_R==1)||(LINEDETECT_L==1)) break; /*ラインを検出したらwhileループを抜ける*/
  }
  stop();
  if(LINEDETECT_R==1)
  {
    result=1;
    AUTOFLASH=4;
    IO.PDR5.BIT.B2=1;   /*右側LEDを点灯*/
    IO.PDR5.BIT.B3=0;
  }
  else if(LINEDETECT_L==1)
  {
    result=2;
    AUTOFLASH=4;
    IO.PDR5.BIT.B2=0;   /*左側LEDを点灯*/
    IO.PDR5.BIT.B3=1;
  }
  else
  {
    result=0;           /*指定した時間が経過して終了した場合*/
  }
   return result;
}

/*********************************************************************/
void INITIALVALUES(void)      /*初期値を設定している関数です。*/
{
  AUTOFLASH=3;                /*パワーオンで表示用LEDを２個とも点灯*/
/*バックグラウンドで動いている位置・姿勢角推定システムに与える初期値  */
  POSITION_X_2=0.0f;          /*ロボットの初期Ｘ座標                  */
  POSITION_Y_2=0.0f;          /*ロボットの初期Ｙ座標                  */
  ATTITUDE_ANGLE0=90.0f;       /*ロボットの初期姿勢角[単位は度(degree)]*/

/*以下の11行は決して書き換えないで下さい。*/
  ATTITUDE_ANGLE=ATTITUDE_ANGLE0;
  POSITION_X=POSITION_X_2;
  POSITION_X_1=POSITION_X_2;
  POSITION_Y=POSITION_Y_2;
  POSITION_Y_1=POSITION_Y_2;
  FAI0=PI*ATTITUDE_ANGLE0/180.0f;
  SINFAI_2=sin(FAI0);
  COSFAI_2=cos(FAI0);
  OMEGA_2=0.0f;
  SUM_R=0.0f;
  SUM_L=0.0f;
}
