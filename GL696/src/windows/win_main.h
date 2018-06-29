#ifndef __win_main_h__
#define __win_main_h__

//----user-defined message---------------------
#define MSG_USER_MASK					(1<<13)
#define MSG_USER_MAIN_UP			(MSG_USER_MASK | MSG_KEY_F2)
#define MSG_USER_MAIN_DOWN		(MSG_USER_MASK | MSG_KEY_F1)


extern sWindow win_main;



#endif
