/*
test_RED.c
Adapté pour ROS2

Nécessite pigpio et ROS2.

Pour compiler (exemple) :
gcc -Wall -pthread -o RED test_RED.c RED.c -lpigpiod_if2 -lrclcpp -lstdc++ -lstd_msgs__rosidl_typesupport_cpp -lstd_msgs__rosidl_generator_cpp -latomic

Assurez-vous que votre CMakeLists.txt gère ces dépendances correctement
et que vous avez bien un environnement ROS2 sourcé.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <pigpiod_if2.h>

// Inclure les headers ROS2
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

// Inclure votre fichier RED.h adapté pour ROS2
#include "RED.h"

/*

Nécessite deux encodeurs (A,B) et (C,D) dont le commun est connecté au GND.

La logique reste la même : les callbacks définis dans RED.c sont appelés
lorsque l'encodeur tourne, et publient sur les topics encodeur_A et encodeur_B.

*/

void fatal(char *fmt, ...)
{
   char buf[128];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   fflush(stderr);

   exit(EXIT_FAILURE);
}

int optGpioA = 23;
int optGpioB = 24;
int optGpioC = 17;
int optGpioD = 27;
int optGlitch = 1000;
int optSeconds = 0;
int optMode = RED_MODE_DETENT;
char *optHost   = NULL;
char *optPort   = NULL;

void cbf1(int pos)
{
   // Callback encodeur A (si besoin de log)
   // printf("Encodeur A : %d\n", pos);
}

void cbf2(int pos)
{
   // Callback encodeur B (si besoin de log)
   // printf("Encodeur B : %d\n", pos);
}

int main(int argc, char *argv[])
{
   // Initialisation ROS2
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("encodeurs");

   // Création des publishers ROS2
   auto encodera_pub = node->create_publisher<std_msgs::msg::Int32>("encodeur_A", 10);
   auto encoderb_pub = node->create_publisher<std_msgs::msg::Int32>("encodeur_B", 10);

   if ((optGpioA < 0) || (optGpioB < 0) || (optGpioA == optGpioB))
   {
      fprintf(stderr, "Erreur: Les GPIO pour l'encodeur A ne sont pas valides!\n");
      return 1;
   }
   
   if ((optGpioC < 0) || (optGpioD < 0) || (optGpioC == optGpioD))
   {
      fprintf(stderr, "Erreur: Les GPIO pour l'encodeur B ne sont pas valides!\n");
      return 1;
   }

   int pi = pigpio_start(optHost, optPort); // connexion au daemon pigpio
   if (pi < 0)
   {
      fprintf(stderr, "Erreur: Impossible de se connecter à pigpio (Daemon non lancé?)\n");
      return 1;
   }

   // On suppose que vous avez modifié RED.c/.h pour utiliser
   // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
   // au lieu de ros::Publisher* dans la structure RED_t.
   RED_t *renc = RED(pi, optGpioA, optGpioB, optGpioC, optGpioD, optMode, cbf1, cbf2, encodera_pub, encoderb_pub);
   RED_set_glitch_filter(renc, optGlitch);

   // Boucle principale
   // Contrairement à ROS1, on pourrait utiliser rclcpp::spin() pour gérer des callbacks ROS,
   // mais ici les callbacks sont gérées par pigpio. On fait juste dormir.
   // Si vous avez besoin d'appels ROS, utilisez rclcpp::spin(node).

   while (rclcpp::ok())
   {
      // On dort 60s entre chaque check
      // Les callbacks pigpio fonctionneront en arrière-plan.
      sleep(60);
      // Si besoin, on peut faire rclcpp::spin_some(node); pour traiter
      // d'éventuelles actions ROS (timers, etc.).
   }

   RED_cancel(renc); 
   pigpio_stop(pi);
   rclcpp::shutdown();
   return 0;
}
