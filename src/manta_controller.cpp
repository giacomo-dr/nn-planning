// manta_controller.cpp
// Author: Giacomo Del Rio
// Date: 12 Apr 2017

#include "manta_controller.h"


MantaController::MantaController( VRepClient& vrep ) : vrep(vrep)
{
    motor_handle = vrep.get_handle( "motor_joint" );
    steer_handle = vrep.get_handle( "steer_joint" );
    base_handle = vrep.get_handle( "Manta" );
}

VRepClient& MantaController::get_client() const{
    return vrep;
}

void MantaController::set_velocity( double mt_sec ){
    vrep.set_joint_target_velocity( motor_handle, mt_sec );
}

void MantaController::set_steer( double angle ){
    vrep.set_joint_target_position( steer_handle, angle );
}

void MantaController::set_position( double x, double y, double z ){
    vrep.set_object_position( base_handle, x, y, z );
}

void MantaController::set_orientation( double a, double b, double g ){
    vrep.set_object_orientation( base_handle, a, b, g );
}

void MantaController::set_pose( Point3D pos, Point3D orient ){
    vrep.set_dynamic_object_pose( base_handle, pos, orient );
}

void MantaController::register_for_getting_pose( int delay_ms ){
    vrep.register_for_position( base_handle, delay_ms );
    vrep.register_for_orientation( base_handle, delay_ms );
    get_position();     // Call it already to avoid garbage on first call
    get_orientation();  //                   "
}

void MantaController::unregister_for_getting_pose(){
    vrep.unregister_for_position( base_handle );
    vrep.unregister_for_orientation( base_handle );
}

Point3D MantaController::get_position(){
    return vrep.get_object_position( base_handle );
}

Point3D MantaController::get_orientation(){
    return vrep.get_object_orientation( base_handle );
}