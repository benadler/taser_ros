//-----------------------------------------------
//File: Cmd_MotCtrl.h
//Author: Oliver Barth / Neobotix
//-----------------------------------------------
#ifndef CMD_MOTCTRL_INCLUDEDEF_H
#define CMD_MOTCTRL_INCLUDEDEF_H
//-----------------------------------------------
enum Cmd_MotCtrl
{
	CMD_MOTCTRL_CONNECT,					// 0
	CMD_MOTCTRL_DISCONNECT,				// 1

	CMD_MOTCTRL_SETDIGOUT,				// 2
	CMD_MOTCTRL_GETDIGIN,					// 3
	CMD_MOTCTRL_GETANALOGIN,			// 4

	CMD_MOTCTRL_SETMOTIONTYPE,		// 5

	CMD_MOTCTRL_GETPOSVEL,				// 6
	CMD_MOTCTRL_GETSTATUS,				// 7

	CMD_MOTCTRL_DISABLEBRAKE,			// 8
	CMD_MOTCTRL_ENABLEMOTOR,			// 9
	CMD_MOTCTRL_SYNCHMOTOR,				// 10
	CMD_MOTCTRL_ENABLECOMM,				// 12

	CMD_MOTCTRL_SETCMDVAL,				// 13

	CMD_MOTCTRL_SETCTRLPARA,			// 14
	CMD_MOTCTRL_GETCTRLPARA,			// 15

	CMD_MOTCTRL_SETPOSCTRL,				// 16
	CMD_MOTCTRL_GETPOSCTRL,				// 17

	CMD_MOTCTRL_SETEMSTOP,				// 18
	CMD_MOTCTRL_RESETEMSTOP,			// 19

	CMD_MOTCTRL_ERROR_STOPMOTION,	// 20
	CMD_MOTCTRL_UNKNOWN						// 21
};
//-----------------------------------------------
#endif
