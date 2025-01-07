/*
RED.h
Adapté pour ROS2
2015-11-18
Public Domain
*/

#ifndef RED_H
#define RED_H

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

typedef void (*RED_CB_t)(int);

struct _RED_s;

typedef struct _RED_s RED_t;

#define RED_MODE_DETENT 0
#define RED_MODE_STEP   1

/*
RED démarre un encodeur rotatif sur le Pi 'pi' avec les GPIO gpioA,
gpioB, gpioC, gpioD, et mode 'mode'. Le mode détermine si l'on rapporte
chaque transition ou seulement les détentes complètes.

Si cb_func1 ou cb_func2 n'est pas NULL, il sera appelé à chaque changement
de position avec la nouvelle position.

La position courante peut être lue avec RED_get_position et définie avec
RED_set_position.

RED_set_glitch_filter peut être utilisé pour filtrer les rebonds mécaniques.
Par défaut, un glitch filter de 1000 microsecondes est utilisé.

À la fin du programme, l'encodeur doit être annulé avec RED_cancel pour
libérer les ressources système.
*/

RED_t *RED(int pi,
           int gpioA,
           int gpioB,
           int gpioC,
           int gpioD,
           int mode,
           RED_CB_t cb_func1,
           RED_CB_t cb_func2,
           rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr my_pub1,
           rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr my_pub2);

void RED_cancel(RED_t *renc);

void RED_set_glitch_filter(RED_t *renc, int glitch);

void RED_set_position(RED_t *renc, int position);

int RED_get_position(RED_t *renc);

#endif
