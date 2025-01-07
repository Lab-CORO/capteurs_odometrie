/*
RED.c
Adapté pour ROS2
2015-11-18
Public Domain
Modifié par J-Philippe Roberge - janvier 2022
*/

#include <stdio.h>
#include <stdlib.h>
#include <pigpiod_if2.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

#include "RED.h"

/* PRIVATE ---------------------------------------------------------------- */
using namespace std;
struct _RED_s
{
   int pi;
   int gpioA;
   int gpioB;
   int gpioC;
   int gpioD;
   RED_CB_t cb1;
   RED_CB_t cb2;
   int cb_id_a;
   int cb_id_b;
   int cb_id_c;
   int cb_id_d;
   int levA;
   int levB;
   int levC;
   int levD;
   int oldState1;
   int oldState2;
   int glitch1;
   int glitch2;
   int mode1;
   int mode2;
   int step1;
   int step2;
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_a;
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_b;
};

static int transits[16]=
{
/* 0000 0001 0010 0011 0100 0101 0110 0111 */
      0,  -1,   1,   0,   1,   0,   0,  -1,
/* 1000 1001 1010 1011 1100 1101 1110 1111 */
     -1,   0,   0,   1,   0,   1,  -1,   0
};

static void _cb1(
   int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   RED_t *self=(RED_t*)user;
   int newState, inc, detent;
   std_msgs::msg::Int32 msg;

   if (level != PI_TIMEOUT)
   {
      if (gpio == self->gpioA)
         self->levA = level;
      else
         self->levB = level;

      newState = (self->levA << 1) | self->levB;
      inc = transits[(self->oldState1 << 2) | newState];

      if (inc)
      {
         self->oldState1 = newState;
         detent = self->step1 / 4;
         self->step1 += inc;

         if (self->cb1)
         {
            if (self->mode1 == RED_MODE_DETENT)
            {
               if (detent != (self->step1 / 4))
               {
                  (self->cb1)(self->step1 / 4);
                  msg.data = (self->step1 / 4);
                  if (self->enc_a)
                     self->enc_a->publish(msg);
               }
            }
            else
            {
               (self->cb1)(self->step1);
               msg.data = self->step1;
               if (self->enc_a)
                  self->enc_a->publish(msg);
            }
         }
      }
   }
}


static void _cb2(
   int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   RED_t *self=(RED_t*)user;
   int newState, inc, detent;
   std_msgs::msg::Int32 msg;

   if (level != PI_TIMEOUT)
   {
      if (gpio == self->gpioC)
         self->levC = level;
      else
         self->levD = level;

      newState = (self->levC << 1) | self->levD;
      inc = transits[(self->oldState2 << 2) | newState];

      if (inc)
      {
         self->oldState2 = newState;
         detent = self->step2 / 4;
         self->step2 += inc;

         if (self->cb2)
         {
            if (self->mode2 == RED_MODE_DETENT)
            {
               if (detent != (self->step2 / 4))
               {
                  (self->cb2)(self->step2 / 4);
                  msg.data = (self->step2 / 4);
                  if (self->enc_b)
                     self->enc_b->publish(msg);
               }
            }
            else
            {
               (self->cb2)(self->step2);
               msg.data = self->step2;
               if (self->enc_b)
                  self->enc_b->publish(msg);
            }
         }
      }
   }
}

/* PUBLIC ----------------------------------------------------------------- */

RED_t *RED(int pi, int gpioA, int gpioB, int gpioC, int gpioD, int mode, RED_CB_t cb_func1, RED_CB_t cb_func2,
           rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr my_pub1,
           rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr my_pub2)
{
   
   RED_t *self = (RED_t*)malloc(sizeof(RED_t));
   if (!self) return NULL;

   self->pi = pi;
   self->gpioA = gpioA;
   self->gpioB = gpioB;
   self->gpioC = gpioC;
   self->gpioD = gpioD;
   self->mode1 = mode;
   self->mode2 = mode;
   self->cb1 = cb_func1;
   self->cb2 = cb_func2;
   self->levA=0;
   self->levB=0;
   self->levC=0;
   self->levD=0;
   self->step1 = 0;
   self->step2 = 0;
   self->enc_a = my_pub1;
   self->enc_b = my_pub2;

   set_mode(pi, gpioA, PI_INPUT);
   set_mode(pi, gpioB, PI_INPUT);
   set_mode(pi, gpioC, PI_INPUT);
   set_mode(pi, gpioD, PI_INPUT);

   /* pull up is needed as encoder common is grounded */
   set_pull_up_down(pi, gpioA, PI_PUD_UP);
   set_pull_up_down(pi, gpioB, PI_PUD_UP);
   set_pull_up_down(pi, gpioC, PI_PUD_UP);
   set_pull_up_down(pi, gpioD, PI_PUD_UP);

   self->glitch1 = 1000;
   self->glitch2 = 1000;

   set_glitch_filter(pi, gpioA, self->glitch1);
   set_glitch_filter(pi, gpioB, self->glitch1);
   set_glitch_filter(pi, gpioC, self->glitch2);
   set_glitch_filter(pi, gpioD, self->glitch2);

   self->oldState1 = (gpio_read(pi, gpioA) << 1) | gpio_read(pi, gpioB);
   self->oldState2 = (gpio_read(pi, gpioC) << 1) | gpio_read(pi, gpioD);

   /* monitor encoder level changes */
   self->cb_id_a = callback_ex(pi, gpioA, EITHER_EDGE, _cb1, self);
   self->cb_id_b = callback_ex(pi, gpioB, EITHER_EDGE, _cb1, self);
   self->cb_id_c = callback_ex(pi, gpioC, EITHER_EDGE, _cb2, self);
   self->cb_id_d = callback_ex(pi, gpioD, EITHER_EDGE, _cb2, self);
   return self;
}

void RED_cancel(RED_t *self)
{
   if (self)
   {
      if (self->cb_id_a >= 0)
      {
         callback_cancel(self->cb_id_a);
         self->cb_id_a = -1;
      }

      if (self->cb_id_b >= 0)
      {
         callback_cancel(self->cb_id_b);
         self->cb_id_b = -1;
      }
      
      if (self->cb_id_c >= 0)
      {
         callback_cancel(self->cb_id_c);
         self->cb_id_c = -1;
      }
      
      if (self->cb_id_d >= 0)
      {
         callback_cancel(self->cb_id_d);
         self->cb_id_d = -1;
      }

      set_glitch_filter(self->pi, self->gpioA, 0);
      set_glitch_filter(self->pi, self->gpioB, 0);
      set_glitch_filter(self->pi, self->gpioC, 0);
      set_glitch_filter(self->pi, self->gpioD, 0);

      free(self);
   }
}

void RED_set_position(RED_t *self, int position)
{
   if (self->mode1 == RED_MODE_DETENT)
      self->step1 = position * 4;
   else
      self->step1 = position;
      
   if (self->mode2 == RED_MODE_DETENT)
      self->step2 = position * 4;
   else
      self->step2 = position;
}

int RED_get_position(RED_t *self)
{
   if (self->mode1 == RED_MODE_DETENT)
      return self->step1 / 4;
   else
      return self->step1;

   // Note : ce code original ne sera jamais atteint après le return.
   // Il faudrait corriger la logique si on veut retourner le mode2 également.
   // Pour l'instant, on laisse tel quel, comme dans le code d'origine.
   if (self->mode2 == RED_MODE_DETENT)
      return self->step2 / 4;
   else
      return self->step2;
}

void RED_set_glitch_filter(RED_t *self, int glitch)
{
   if (glitch >= 0)
   {
      if (self->glitch1 != glitch)
      {
         self->glitch1 = glitch;
         set_glitch_filter(self->pi, self->gpioA, glitch);
         set_glitch_filter(self->pi, self->gpioB, glitch);
      }
      if (self->glitch2 != glitch)
      {
         self->glitch2 = glitch;
         set_glitch_filter(self->pi, self->gpioC, glitch);
         set_glitch_filter(self->pi, self->gpioD, glitch);
      }
   }
}
