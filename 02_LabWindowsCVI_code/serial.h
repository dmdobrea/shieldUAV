/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  CONFIG                           1
#define  CONFIG_COMPORT                   2       /* control type: slide, callback function: (none) */
#define  CONFIG_BAUDRATE                  3       /* control type: slide, callback function: (none) */
#define  CONFIG_PARITY                    4       /* control type: slide, callback function: (none) */
#define  CONFIG_DATABITS                  5       /* control type: slide, callback function: (none) */
#define  CONFIG_STOPBITS                  6       /* control type: slide, callback function: (none) */
#define  CONFIG_INPUTQ                    7       /* control type: numeric, callback function: InputQCallBack */
#define  CONFIG_OUTPUTQ                   8       /* control type: numeric, callback function: (none) */
#define  CONFIG_CTSMODE                   9       /* control type: binary, callback function: (none) */
#define  CONFIG_XMODE                     10      /* control type: binary, callback function: (none) */
#define  CONFIG_TIMEOUT                   11      /* control type: numeric, callback function: (none) */
#define  CONFIG_CLOSECONFIG               12      /* control type: command, callback function: CloseConfigCallback */
#define  CONFIG_TIMEOUT_MSG1              13      /* control type: textMsg, callback function: (none) */
#define  CONFIG_OUTQSIZE_MSG              14      /* control type: textMsg, callback function: (none) */

#define  SERIAL                           2
#define  SERIAL_CONFIG                    2       /* control type: command, callback function: ConfigCallBack */
#define  SERIAL_ERROR                     3       /* control type: command, callback function: ErrorCallBack */
#define  SERIAL_COMSTATUS                 4       /* control type: command, callback function: ComStatusCallBack */
#define  SERIAL_SENDTERM                  5       /* control type: ring, callback function: (none) */
#define  SERIAL_TBOX_SEND                 6       /* control type: textBox, callback function: (none) */
#define  SERIAL_SENDBYTE                  7       /* control type: numeric, callback function: (none) */
#define  SERIAL_SENDMODE                  8       /* control type: binary, callback function: SendModeCallBack */
#define  SERIAL_BYTES                     9       /* control type: numeric, callback function: (none) */
#define  SERIAL_SEND                      10      /* control type: command, callback function: SendCallBack */
#define  SERIAL_READ_COUNT                11      /* control type: numeric, callback function: (none) */
#define  SERIAL_READTERM                  12      /* control type: ring, callback function: (none) */
#define  SERIAL_TBOX_READ                 13      /* control type: textBox, callback function: (none) */
#define  SERIAL_READ                      14      /* control type: command, callback function: ReadCallBack */
#define  SERIAL_CLEARBOX                  15      /* control type: command, callback function: ClearBoxCallBack */
#define  SERIAL_GETOUTQ                   16      /* control type: command, callback function: GetOutQCallBack */
#define  SERIAL_GETINQ                    17      /* control type: command, callback function: GetInQCallBack */
#define  SERIAL_FLUSHOUTQ                 18      /* control type: command, callback function: FlushOutQCallBack */
#define  SERIAL_FLUSHINQ                  19      /* control type: command, callback function: FlushInCallBack */
#define  SERIAL_QUIT                      20      /* control type: command, callback function: QuitCallBack */
#define  SERIAL_COMMANDBUTTON_2           21      /* control type: command, callback function: Send1000 */
#define  SERIAL_COMMANDBUTTON             22      /* control type: command, callback function: Send2000 */
#define  SERIAL_PWM2                      23      /* control type: scale, callback function: fPWM2 */
#define  SERIAL_PWM1                      24      /* control type: scale, callback function: fPWM1 */
#define  SERIAL_RINGSLIDE_LED3            25      /* control type: slide, callback function: fLED3 */
#define  SERIAL_RINGSLIDE_LED2            26      /* control type: slide, callback function: fLED2 */
#define  SERIAL_RINGSLIDE_LED1            27      /* control type: slide, callback function: fLED1 */
#define  SERIAL_COMMANDBUTTON_SEND        28      /* control type: command, callback function: fSend_ALL */
#define  SERIAL_DECORATION                29      /* control type: deco, callback function: (none) */
#define  SERIAL_DECORATION_2              30      /* control type: deco, callback function: (none) */
#define  SERIAL_TRANSMIT_MSG              31      /* control type: textMsg, callback function: (none) */
#define  SERIAL_RECEIVE_MSG               32      /* control type: textMsg, callback function: (none) */
#define  SERIAL_CONFIG_MSG                33      /* control type: textMsg, callback function: (none) */
#define  SERIAL_RCV_HELP_MSG              34      /* control type: textMsg, callback function: (none) */
#define  SERIAL_TRANS_HELP_MSG            35      /* control type: textMsg, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK ClearBoxCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK CloseConfigCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ComStatusCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ConfigCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ErrorCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fLED1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fLED2(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fLED3(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK FlushInCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK FlushOutQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fPWM1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fPWM2(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fSend_ALL(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK GetInQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK GetOutQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK InputQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK QuitCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ReadCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Send1000(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Send2000(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SendCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SendModeCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
