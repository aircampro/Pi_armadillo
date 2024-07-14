// -*- C++ -*-
/*!
 * @file  KobukiAutoMove.cpp
 * @brief Example creation of Kobuki auto move component
 * @date 12/07/24
 *
 * $Id$
 */

#include <coil/Time.h>
#include "KobukiAutoMove.h"

#include <stdio.h>
#include <iostream>
#include <random>

// Module specification
// <rtc-template block="module_spec">
static const char* kobukiautomove_spec[] =
  {
    "implementation_id", "KobukiAutoMove",
    "type_name",         "KobukiAutoMove",
    "description",       "Kobuki auto move component",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
KobukiAutoMove::KobukiAutoMove(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_bumperIn("bumper", m_bumper),
    m_targetVelocityOut("targetVelocity", m_targetVelocity)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
KobukiAutoMove::~KobukiAutoMove()
{
}
 
/*!
 * @perform this on initialise
 */
RTC::ReturnCode_t KobukiAutoMove::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("bumper", m_bumperIn);
  
  // Set OutPort buffer
  addOutPort("targetVelocity", m_targetVelocityOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/* perform on finalize 
RTC::ReturnCode_t KobukiAutoMove::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/* perform on start-up
RTC::ReturnCode_t KobukiAutoMove::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/* perform on shutdown
RTC::ReturnCode_t KobukiAutoMove::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
 
/*!
 * @do on activation
 */
RTC::ReturnCode_t KobukiAutoMove::onActivated(RTC::UniqueId ec_id)
{
	m_bumper_flg = false;
	m_run_flg = true;
	m_i,m_x = 0;
	m_z = -1;
        m_y = X_DISTANCE;
        m_yy = 0;
        m_xx = 0;
	
	kobuki_straight(0.25);	

  return RTC::RTC_OK;
}

/*!
 * @do on de-activation
 */
RTC::ReturnCode_t KobukiAutoMove::onDeactivated(RTC::UniqueId ec_id)
{
        m_run_flg = false;
	kobuki_stop();

	return RTC::RTC_OK;
}
 
/*!
 * @perform on execute
 */
RTC::ReturnCode_t KobukiAutoMove::onExecute(RTC::UniqueId ec_id)
{
	int which_bumpers = 0;                                       // this gets set if it finds a bumper action that we react to
	
    // check bumpers for collision and move accordingly	
	if ((m_bumperIn.isNew()) && (m_yy < Y_DIST_STEPS))
	{
            m_bumperIn.read();
            which_bumpers = check_bumper_status();                          // check bumpers for obstacles and do a turn to avoid it
            if ((which_bumpers & (1<<1)) || ((1<<6) & which_bumpers))  {    // center bumper or center cliff
                // choose either left or right randomly to avoid 2 bots colliding with same actions of avoidance
		if ( m_i >= NUM_X_LENGTHS) {                                // we are moving back along the y axis
                    go_right_x(2, 90);                               // make a step right to avoid the oncoming obstacle
                    m_xx = m_xx + 2;				
		} else {                                             // we are moving up and down x direction
                    std::random_device rd;
                    std::mt19937 mt(rd());                           // marsenne twister random number generator
		    std::uniform_int_distribution<int> dist(0, 1);   // a number from 0 to 1
		    if (dist(mt)==0) {                               // even choice
                        go_right_y(2, 90);	
                        m_yy = m_yy + 2;				
                    } else {                                         // odd choice
                        go_left_y(2, 90);
                        m_yy = m_yy - 2;
                    } 
               }
           } else if (which_bumpers & (1<<2)) {                      // left bumper 
              if ((m_x < X_DISTANCE) && (m_z == -1)) {              // going up in x direction
                  go_right_y(2, 90);                                // turn right
                  m_yy = m_yy + 2;	
              } else if ((m_x >= X_DISTANCE) && (m_z > 0)) {        // going down in x direction
                  go_left_y(2, 90);                                 // turn left	
                  m_yy = m_yy - 2;					
              }				
           } else if (which_bumpers & (1<<0)) {                      // right bumper 
              if ((m_x < X_DISTANCE) && (m_z == -1)) {              // going up in x direction
                go_left_y(2, 90);                                 // turn left
                m_yy = m_yy - 2;	
            } else if ((m_x >= X_DISTANCE) && (m_z > 0)) {        // going down in x direction
                go_right_y(2, 90);                                // turn right	
                m_yy = m_yy + 2;					
            }
        }			
    } else if (m_yy >= Y_DIST_STEPS) {                            // we moved out by more than the threshold
         std::cout << " maximum avoidance steps made" << std::endl;
         kobuki_stop();	                                          // just stop it might work	
    }

    // while bot is running through the square area	iterate to go up and down the area
    // i think this runs on a continous timer so no need for making a while loop if not change to have above included in while(m_run_flg)
    //
    if (m_run_flg)
    {
        if (m_i < NUM_X_LENGTHS)                                     // iterate 10 times back and forth in rows 
	{                  
            if (m_x < X_DISTANCE)                                    // first move X_DISTANCE forward
            {
		if (m_xx > 0) {
                    go_left_x(m_xx, 90);                             // move back to start point if we moved off it to come back
                    m_xx = 0;
                }
		kobuki_straight(m_x / -100.0f);
                wait_for_ms(40.0f);
		++m_x;
		if (m_x == X_DISTANCE) m_z = m_x;
            }
            else if (m_z > 0)                                        // then move X_DISTANCE back
            {
		kobuki_straight(m_z / -100.0);
                wait_for_ms(40.0f);
		--m_z;
		if ((m_z==0) && (m_x==X_DISTANCE)) m_y = 0; 
             }
             else if (m_y == 0)                                       // after that turn right 20
             {
                go_right_y(Y_DIST_STEPS-m_yy, 90);
                m_yy = 0;
                wait_for_ms(40.0f);
                ++m_y;
		if (((m_z==0) && (m_x==X_DISTANCE)) && (m_y == 1))
                {
                    ++m_i;
	            m_x = 0;
                    m_z = -1;
                    m_y = X_DISTANCE;						
                }
            }
        }
        else if ( m_i >= NUM_X_LENGTHS)                                        // done 10 iterations of above sequence
        {			
            go_left_y((Y_DIST_STEPS*NUM_X_LENGTHS), 90);
            wait_for_ms(40.0f);
	    m_i,m_x = 0;
            m_z = -1;
            m_y = X_DISTANCE;	
        }			
     }

     return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KobukiAutoMove::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiAutoMove::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiAutoMove::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiAutoMove::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KobukiAutoMove::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*!
 * @check the bumper status and return an integer if there is one
 */
int KobukiAutoMove::check_bumper_status(void)
{
	bool flg = true;
        int bump_stat = 0;
	
	if (m_bumper.data[RIGHT_BUMPER])
	{
		output_msg("RIGHT_BUMPER");		// バンパーの接触
                bump_stat = bump_stat | (1<<0);
	}
	if (m_bumper.data[CENTER_BUMPER])
	{
		output_msg("CENTER_BUMPER");
                bump_stat = bump_stat | (1<<1);
	}
	if (m_bumper.data[LEFT_BUMPER])
	{	
		output_msg("LEFT_BUMPER");
                bump_stat = bump_stat | (1<<2);
	}
	if (m_bumper.data[RIGHT_WHEEL_DROP])
	{
		output_msg("RIGHT_WHEEL_DROP");		// 車輪の落下
                bump_stat = bump_stat | (1<<3);
	}
	if (m_bumper.data[LEFT_WHEEL_DROP])
	{
		output_msg("LEFT_WHEEL_DROP");
                bump_stat = bump_stat | (1<<4);
	}
	if (m_bumper.data[RIGHT_CLIFF])
	{
		output_msg("RIGHT_CLIFF");				// 下方向の赤外線センサの反応 
                bump_stat = bump_stat | (1<<5);
	}
	if (m_bumper.data[CENTER_CLIFF])
	{
		output_msg("CENTER_CLIFF");
                bump_stat = bump_stat | (1<<6);
	}
	if (m_bumper.data[LEFT_CLIFF])
	{
		output_msg("LEFT_CLIFF");
                bump_stat = bump_stat | (1<<7);
	}
	else
	{
		// 障害物なし
		flg = false;
	}
	m_bumper_flg = flg;

	return bump_stat;
}

/*!
 * wair for ms time period
 */
void wait_for_ms( float tim ) 
{
#ifdef _WIN32
    Sleep(static_cast<int>(tim));
#else
    coil::sleep(coil::TimeValue(tim/100.0f));		// 40msec
#endif
}

/*!
 * @print message text
 */
void KobukiAutoMove::output_msg(std::string msg)
{
	std::cout << "m_bumper::" << msg << std::endl;

	return;
}

/*!
 * @move
 */
void KobukiAutoMove::kobuki_move(MOVE_PARAM prm)
{
	m_targetVelocity.data.vx = prm.vx;
	m_targetVelocity.data.vy = prm.vy;
	m_targetVelocity.data.va = prm.va;
	m_targetVelocityOut.write();

	return;
}

/*!
 * @stop
 */
void KobukiAutoMove::kobuki_stop(void)
{
	MOVE_PARAM prm;
	prm.vx = 0;
	prm.vy = 0;
	prm.va = 0;
	kobuki_move(prm);

	return;
}

/*!
 * @kobuki_straight x direction
 */
void KobukiAutoMove::kobuki_straight(double vel)
{
	MOVE_PARAM prm;
	prm.vx = vel;
	prm.vy = 0;
	prm.va = 0;
	kobuki_move(prm);

	return;
}

/*!
 * @kobuki_side y direction
 */
void KobukiAutoMove::kobuki_side(double vel)
{
	MOVE_PARAM prm;
	prm.vx = 0;
	prm.vy = vel;
	prm.va = 0;
	kobuki_move(prm);

	return;
}

/*!
 * @kobuki_turn angle direction
 */
void KobukiAutoMove::kobuki_turn(double vel)
{
	MOVE_PARAM prm;
	prm.vx = 0;
	prm.vy = 0;
	prm.va = vel;
	kobuki_move(prm);

	return;
}

/*!
 * @turn right in y direction
 */
void KobukiAutoMove::go_right_y(int steps, int turns)
{
	for (int s=0;s<turns;++s) {
        kobuki_turn(s / 20.0f);                  // go right 90
	}
	for (int s=0;s<steps;++s) {
        kobuki_side(s/100.0f);                   // go y direction	
    }
	for (int s=turns;s<0;--s) {
        kobuki_turn(s / -20.0f);                  // turn back
	}
	return;
}

/*!
 * @turn right in x direction
 */
void KobukiAutoMove::go_right_x(int steps, int turns)
{
	for (int s=0;s<turns;++s) {
        kobuki_turn(s / 20.0f);                  // go right 90
	}
	for (int s=0;s<steps;++s) {
        kobuki_straight(s/100.0f);              // go x direction	
        }
	for (int s=turns;s<0;--s) {
        kobuki_turn(s / -20.0f);                  // turn back
	}
	return;
}

/*!
 * @turn left in y direction
 */
void KobukiAutoMove::go_left_y(int steps, int turns)
{
	for (int s=0;s<turns;++s) {
        kobuki_turn(s / -20.0f);                 // go left 90
	}
	for (int s=0;s<step;++s) {
        kobuki_side(s/100.0f);                   // go y direction	
        }
	for (int s=turns;s<0;--s) {
        kobuki_turn(s / 20.0f);                  // turn back
	}
}

/*!
 * @turn left in x direction
 */
void KobukiAutoMove::go_left_x(int steps, int turns)
{
	for (int s=0;s<turns;++s) {
        kobuki_turn(s / -20.0f);                 // go left 90
	}
	for (int s=0;s<step;++s) {
        kobuki_straight(s/100.0f);                   // go y direction	
        }
	for (int s=turns;s<0;--s) {
        kobuki_turn(s / 20.0f);                  // turn back
	}
}

extern "C"
{
 
  void KobukiAutoMoveInit(RTC::Manager* manager)
  {
    coil::Properties profile(kobukiautomove_spec);
    manager->registerFactory(profile, RTC::Create<KobukiAutoMove>, RTC::Delete<KobukiAutoMove>);
  }
  
};