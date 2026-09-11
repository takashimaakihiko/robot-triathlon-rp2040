/******************************************************************
   File–¼Frobot.c

  "ƒƒ{ƒbƒgƒgƒ‰ƒCƒAƒXƒƒ“—p•W€ƒƒ{ƒbƒg"‚Ì§ŒäƒvƒƒOƒ‰ƒ€ƒeƒ“ƒvƒŒ[ƒg

                                     by YOU-YOU ROBOTICS (2002/8/1)
*******************************************************************/

#include <robot.h>    /*‚¢‚Â‚à‚±‚Ìƒwƒbƒ_[‚ðŽg‚¢‚Ü‚·B3664F.H‚ÆMATH.H
                        ‚Írobot.h‚Ì’†‚ÅƒCƒ“ƒNƒ‹[ƒh‚µ‚Ä‚¢‚Ü‚·‚Ì‚ÅA‚±
                        ‚±‚Å‘‚­•K—v‚Í‚ ‚è‚Ü‚¹‚ñB                   */

void program0(void)
{
  /*SWDATA=0‚Ì’l‚ÅŽÀs‚³‚¹‚éŠÖ”‚Ì’†g‚ð‘‚¢‚Ä‰º‚³‚¢B*/
  
}

void program1(void)
{
  /*SWDATA=1‚Ì’l‚ÅŽÀs‚³‚¹‚éŠÖ”‚Ì’†g‚ð‘‚¢‚Ä‰º‚³‚¢B*/
  
}

void program2(void)
{
  /*SWDATA=2‚Ì’l‚ÅŽÀs‚³‚¹‚éŠÖ”‚Ì’†g‚ð‘‚¢‚Ä‰º‚³‚¢B*/
  
}

void program3(void)
{
  /*SWDATA=3‚Ì’l‚ÅŽÀs‚³‚¹‚éŠÖ”‚Ì’†g‚ð‘‚¢‚Ä‰º‚³‚¢B*/
  
}

main(void)	/*‚b‚ÌƒƒCƒ“ŠÖ”*/
{

  switch(SWDATA)  /*‚Qƒrƒbƒg‚ÌƒfƒBƒbƒvƒXƒCƒbƒ`‚Ì’l‚É‚æ‚Á‚ÄŽÀs‚·‚éƒvƒƒOƒ‰ƒ€‚ð‘I‘ð‚µ‚Ü‚·B*/
  {
    case 0:  program0();  /*SWDATA=0‚Ì‚Æ‚«Aprogram0()‚ðŽÀs*/
             break;
    case 1:  program1();  /*SWDATA=1‚Ì‚Æ‚«Aprogram1()‚ðŽÀs*/
             break;
    case 2:  program2();  /*SWDATA=2‚Ì‚Æ‚«Aprogram2()‚ðŽÀs*/
             break;
    default: program3();  /*SWDATA=3‚Ì‚Æ‚«Aprogram3()‚ðŽÀs*/
             break;
  }
}
