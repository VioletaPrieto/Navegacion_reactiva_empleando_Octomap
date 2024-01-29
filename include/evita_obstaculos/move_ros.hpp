#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "octomap/AbstractOcTree.h"
#include "octomap/AbstractOccupancyOcTree.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "fstream"
#include "evita_obstaculos/detecta_obstaculo.hpp"
#include "evita_obstaculos/odometria.hpp"
#include "evita_obstaculos/utilities.hpp"
#include "evita_obstaculos/motion.hpp"
#include "evita_obstaculos/visualization.hpp"



class move_ros
{
private:

    detecta_obstaculo detecta_obstaculo_p;
    odometria odometria_p;
    utilities utilities_p;
    motion motion_p;
    visualization visualization_p;

    ros::NodeHandle n_;  //Interactuar con el sistema de ROS

    ros::Subscriber octomap_sub; 
    ros::Subscriber odometry_sub;
    ros::Publisher vel_pub;
    ros::Publisher visualization_pub;

    octomap::AbstractOcTree *tree;  //Puntero a una estructura AbstractOcTree para almacenar en estructura arbol
    octomap::OcTree *Octree; //Almacena el mapa de cuadrícula de ocupación 3D en un OcTree
    
    nav_msgs::Odometry::ConstPtr odometry;
    
    std::vector<double> pos_actual;
    std::vector<double> orientacion_actual_q;
    double orientacion_actual_E;
    double orientacion_prueba;
    double distancia_final;
    char nueva_posicion_final;
    char continuar;
    bool bool_odom;
    bool octomap_recibido;
    double distancia_punto_destino;
    double distancia_anterior_punto_destino;
    double angulo_punto_destino;
    double dif_angular;
    char cont;
    int interaciones_ejecucion_totales;
    double radio_circunferencia;
    double numPuntos_circunferecnia;
    bool evaluar_recta;
    bool punto_nuevo;
    bool no_punto_valido;


    double radio_ext;
    double radio_int;
    double num_puntos;
    double distancia_calculo;
    double tolerancia_angular;
    double tolerancia_distancia;
    double tolerancia_distancia_final;
    octomap::point3d PuntoFIN3d;
    double pos_fin_x;
    double pos_fin_y;
    double pos_fin_z;
    
    std::vector<octomap::point3d> Vector_puntos_aleatorios;
    std::vector<double> Vector_distancias_pto_final;
    std::vector<octomap::point3d> vector_intermediatePoints;
    std::vector<octomap::point3d> vector_circunferencePoints;

    octomap::point3d pos_actual3d;
    std::vector<double> RandomPoint;
    octomap::point3d RandomPoint3d;
    bool occupation;
    bool ocupacion_punto_destino;
    bool inter_occupation;
    bool circunference_occupation;
    double distancia_pto_final;
    int indice_min_distancia;
    octomap::point3d pos_destino3d;
    geometry_msgs::Twist Comando_velocidad;
    bool no_avanzar;

    
   visualization_msgs::Marker borrar_puntos;
   visualization_msgs::Marker punto_visualizacion;
   visualization_msgs::MarkerArray array_visualizacion;
   visualization_msgs::Marker linea_visualizacion;

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg);
    void odometryCallback(nav_msgs::Odometry::ConstPtr odometry_);
    
public:

    move_ros(ros::NodeHandle n);
    ~ move_ros();

    void run();

};    