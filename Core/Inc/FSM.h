/*
 * FSM.h
 *
 *  Created on: Mar 16, 2025
 *      Author: user
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_
// Function prototypes
void moveTo1Function(int);
void moveLeftFunction(int);
void moveRightFunction(int);
void leftToRightFunction(int);
void rightToLeftFunction(int);
void moveTo2Function(int);
void moveLeft2Function(int);
void moveRight2Function(int);
void moveTo3Function(int);



//static const void (*fn[])(int) = {function1, function2, function3,};

struct State {
	void (*stateFunction)(void); //function to run in current state
	//float arg; // argument to pass into the function pointer
	const struct State *next[2]; //
};
typedef const struct State State_t;

#define LeftState 0
#define RightState 1
#define Duncare 0

//#define MoveToBlock          &fsm[0] //Phase 1
//#define MoveLeft           &fsm[1]
//#define MoveRight           &fsm[2]
//#define LeftToRight       &fsm[3]
//#define RightToLeft       &fsm[4]
//#define MoveTo2       &fsm[5] //Phase 2
//#define MoveLeft2          &fsm[6]
//#define MoveRight2          &fsm[7]
//#define MoveTo3      &fsm[8] //Phase 3 parking phase
/*
State_t fsm[9] = {
    {moveTo1Function,	30,    {MoveLeft, MoveRight, MoveTo1}},    // MoveTo1
    {moveLeftFunction,	30,  {LeftToRight, MoveLeft, MoveTo1}},  // MoveLeft
    {moveRightFunction,	30,  {RightToLeft, MoveRight, MoveTo1}}, // MoveRight
    {leftToRightFunction,30, {MoveTo2, LeftToRight, MoveTo1}},  // LeftToRight
    {rightToLeftFunction,30, {MoveTo2, RightToLeft, MoveTo1}},  // RightToLeft
    {moveTo2Function,	30,    {MoveLeft2, MoveRight2, MoveTo2}},  // MoveTo2
    {moveLeft2Function,	30,  {MoveTo3, MoveLeft2, MoveTo2}},     // MoveLeft2
    {moveRight2Function,30, {MoveTo3, MoveRight2, MoveTo2}},    // MoveRight2
    {moveTo3Function,	30,    {MoveTo3, MoveTo3, MoveTo3}}        // MoveTo3
};*/
/*
State_t fsm[11]={
  {0x03,{ Right1,         Left1,      Right1,     Center    }},   // MoveTo1
  {0x02,{ Left_off1,      Left2,      Right1,     Center    }},   // MoveLeft
  {0x03,{ Left_off1,      Left1,      Right1,     Center    }},   // MoveRight
  {0x02,{ Left_off2,      Left_off2,  Left_off2,  Left_off2 }},   // LeftToRight
  {0x03,{ Left_stop,      Left1,      Right1,     Center    }},   // RightToLeft
  {0x00,{ Left_stop,      Left_stop,  Left_stop,  Center }},   // Left_stop
  {0x01,{ Right_off1,     Left1,      Right2,     Center    }},   // Right1
  {0x03,{ Right_off1,     Left1,      Right1,     Center    }},   // Right2
  {0x01,{ Right_off2,     Right_off2, Right_off2, Right_off2}},   // Right_off1
  {0x03,{ Right_stop,     Left1,      Right1,     Center    }},   // Right_off2
  {0x00,{ Right_stop,     Right_stop, Right_stop, Center}}    // Right_stop
};*/


#endif /* INC_FSM_H_ */
